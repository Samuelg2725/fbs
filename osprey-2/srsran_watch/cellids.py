#!/usr/bin/env python3
"""Read the cells on an EARFCN, follow their neighbour lists, and survey bands.

This file does not decode. It starts the Pluto pdsch_ue, reads a cell, and keeps
the reader running long enough to also catch that cell's SIB4/SIB5 neighbour
lists:

  SIB4  physical cell IDs of other cells on the SAME frequency (co-channel)
  SIB5  the OTHER EARFCNs this cell lists as neighbours, and their PCIs

Two scan styles:
  normal  one pdsch_ue per sync group (0/1/2): up to three Cell IDs per channel,
          plus the SIB4/SIB5 crawl. Thorough but ~3 Pluto set-ups per channel.
  --fast  one pdsch_ue per channel (no -l): locks the strongest cell only, pays
          the Pluto set-up once. Much quicker; one Cell ID per channel, no crawl.

--sweep seeds from a built-in UK band plan. --log writes each Cell ID to a file.

Pluto limits: one RX, 20 MHz, omni, and ~5 s to open/calibrate per launch.
pdsch_ue returns the STRONGEST cell it locks, so weaker co-channel cells are
heard (reported as present) but not decoded. More Cell IDs come from scanning
more channels (crawl / --sweep), not from one frequency.
"""

from __future__ import annotations

import argparse
import os
import re
import select
import signal
import subprocess
import sys
import time
from datetime import datetime

# band, downlink start in tenths of a MHz, EARFCN offset, first, last.
# F_tenths = start + (earfcn - offset).
_BANDS = (
    (1, 21100, 0, 0, 599),
    (3, 18050, 1200, 1200, 1949),
    (7, 26200, 2750, 2750, 3449),
    (8, 9250, 3450, 3450, 3799),
    (20, 7910, 6150, 6150, 6449),
    (28, 7580, 9210, 9210, 9659),
    (38, 25700, 37750, 37750, 38249),
    (40, 23000, 38650, 38650, 39649),
)

# UK downlink EARFCNs across the supported bands, to seed a --sweep. The crawl
# expands this with whatever SIB5 neighbours are discovered.
_SWEEP_PLAN = (
    6200, 6300, 6400,              # Band 20  800 MHz
    3500, 3625, 3750,              # Band 8   900 MHz
    1300, 1500, 1617, 1801, 1900,  # Band 3   1800 MHz
    99, 275, 399, 525,             # Band 1   2100 MHz
    2850, 3100, 3350,              # Band 7   2600 MHz
    9310, 9485,                    # Band 28  700 MHz
    38100,                         # Band 38  2600 TDD
    38750, 39150,                  # Band 40  2300 TDD
)

_PDSCH = "/home/revector/srsRAN_4G/build/lib/examples/pdsch_ue"
_IDENTITY = re.compile(
    r"SIB1 cell identity (\d+)\s+eNB (\d+)\s+sector (\d+)\s+TAC (\d+)\s+PLMN ([\d-]+)"
)
_SIB4 = re.compile(r"SIB4 intra-freq neighbours\s+([\d ]+)")
_SIB5 = re.compile(r"SIB5 inter-freq EARFCN (\d+)(?:\s+PCI\s+([\d ]+))?")
_SCHED = re.compile(r"SIB1 schedules ((?:SIB\d+\s*)+)")
_PCI = re.compile(r"\*?Found Cell_id:\s+(\d+)\s+(FDD|TDD).*PSR=([0-9.]+)")
_MIB = re.compile(r"Decoded MIB")
_RATE = re.compile(r"Setting rx sampling rate ([0-9.]+) MHz")
_MISS = re.compile(r"Could not find any cell")
_GROUP = re.compile(r"SCAN sync group \(N_id_2\) (\d+)")
_SHADOW = re.compile(r"MULTI group (\d+) distinct cells:(.*)")

# --scan-all: use the one-launch binary that decodes all 3 sync groups in a
# single open radio session (osprey-2 pdsch_ue scan build). Off by default so
# the stock per-group launch path still works with an unmodified pdsch_ue.
_SCAN_ALL = False

# --deshadow: set DESHADOW=<reps> for the scan build so it repeats each channel's
# scan, catching cells that share a sync group (shadowed cells). Needs --scan-all.
_DESHADOW = False
_DESHADOW_REPS = 10
# Hard cap on how long the smart sweep will dig one channel, so a channel with a
# stubborn ghost can't run away (one ate 2.5 minutes before this cap).
_DESHADOW_MAX_SECONDS = 90

# A PSS peak this weak is almost always noise/sidelobe, not a cell. We still show
# it, but it does not count as a "cell heard", so it cannot trigger a de-shadow
# dig. Real cells here run PSR 6-8; the ghosts that waste time sit around 2-2.5.
_PRESENT_FLOOR = 4.0

# Set when Ctrl+C is caught mid-channel so the sweep stops but the cells already
# decoded are still reported.
_STOP = False

# SoapySDR radio selection. Overridden from the command line in main(). The
# Pluto is the default; --driver lime switches pdsch_ue to a LimeSDR. Extra
# device args (e.g. rxant=LNAL to force the low-band port) append after the
# driver, comma separated.
_DRIVER = "plutosdr"
_DEVICE_ARGS = ""

# Optional fast PSS triage (--triage). Off by default: with it off the scanner
# behaves exactly as before. _TRIAGE_THRESHOLD <= 0 means "measure only, never
# skip" (calibration). _TRIAGE_WARNED keeps the fallback warning to one line.
_TRIAGE = False
_TRIAGE_THRESHOLD = 6.0
_TRIAGE_WARNED = False

# When --log is given, every printed line is also appended here, so the file
# is the complete run log, not just the decoded Cell IDs.
_LOG = None

# Seconds to wait for MIB after a tower is found before giving up on a group.
# A real cell decodes MIB within ~1-2 s even at low PSR; a sync that never
# decodes MIB is noise/too-weak. 0 disables the early abort (old behaviour).
_WEAK_TIMEOUT = 5.0

# Seconds to wait for SIB1 (the Cell ID) after MIB decodes before giving up.
# A cell at a usable SNR decodes SIB1 within ~1-2 s of MIB; if MIB came but
# SIB1 will not (overload at the wrong gain, or too weak) it never arrives, so
# stop instead of holding the full dwell. 0 disables it.
_SIB1_TIMEOUT = 8.0


def _soapy_device():
    """The -a device string for pdsch_ue, e.g. 'driver=lime,rxant=LNAL'."""
    spec = "driver=%s" % _DRIVER
    if _DEVICE_ARGS:
        spec += "," + _DEVICE_ARGS.strip().lstrip(",")
    return spec


def _triage_channel(hz, gain, earfcn):
    """Fast PSS presence gate before the slow srsRAN scan.

    Returns False only to SKIP a channel that is confidently empty. Returns
    True when a cell is likely present, or when triage could not run (import
    or capture failure) -- in that case the caller does a full scan, so a
    failure never costs a cell. With a threshold <= 0 it measures and prints
    the ratio but never skips (use it to calibrate the threshold).
    """
    global _TRIAGE_WARNED
    try:
        import pss_triage  # noqa: PLC0415  (sits next to this script)
    except Exception as exc:
        if not _TRIAGE_WARNED:
            say("triage unavailable (%s) - running full scans" % exc)
            _TRIAGE_WARNED = True
        return True
    try:
        is_present, ratio = pss_triage.present(_soapy_device(), hz, gain, _TRIAGE_THRESHOLD)
    except Exception as exc:
        if not _TRIAGE_WARNED:
            say("triage capture failed (%s) - running full scans" % exc)
            _TRIAGE_WARNED = True
        return True
    if _TRIAGE_THRESHOLD <= 0:
        say("EARFCN %d  triage: PSS ratio %.1f (measure only, not skipping)" % (earfcn, ratio))
        return True
    if is_present:
        say("EARFCN %d  triage: PSS ratio %.1f >= %.1f, scanning" % (earfcn, ratio, _TRIAGE_THRESHOLD))
        return True
    say("EARFCN %d  triage: PSS ratio %.1f < %.1f, skipping (empty)" % (earfcn, ratio, _TRIAGE_THRESHOLD))
    return False


def downlink_hz(earfcn):
    """Hz of one downlink EARFCN, or None when the table above does not cover it."""
    for _band, start, offset, first, last in _BANDS:
        if first <= earfcn <= last:
            return (start + (earfcn - offset)) * 100_000
    return None


def mhz_label(earfcn):
    hz = downlink_hz(earfcn)
    return "%.1f MHz" % (hz / 1e6) if hz is not None else "outside band table"


def parse_gains(spec):
    """Comma-separated gains like '45,68' -> [45.0, 68.0], or None if empty."""
    if not spec:
        return None
    gains = [float(part) for part in spec.split(",") if part.strip()]
    return gains or None


def say(text):
    line = datetime.now().strftime("%Y-%m-%d %H:%M:%S  ") + text
    print(line, flush=True)
    if _LOG:
        try:
            with open(_LOG, "a", encoding="utf-8") as handle:
                handle.write(line + "\n")
        except OSError:
            pass


# UK operator names by MNC (MCC 234/235). Used to label the final report.
_UK_MNC = {
    "02": "O2", "10": "O2", "11": "O2",
    "15": "Vodafone",
    "20": "Three", "25": "Three",
    "30": "EE", "33": "EE", "34": "EE",
}


def operator_label(line):
    """Operator name parsed from a decoded cell line's PLMN, or '' if unknown."""
    match = re.search(r"PLMN\s+(\d+)-(\d+)", line or "")
    if not match:
        return ""
    mcc, mnc = match.group(1), match.group(2)
    if mcc in ("234", "235"):
        return _UK_MNC.get(mnc) or _UK_MNC.get(mnc.zfill(2)) or ""
    return ""


def final_report(found_cells):
    """Print every decoded Cell ID with its full details at the end.

    found_cells maps a 28-bit cell id to its decoded detail line. say() also
    writes each line to the --log file, so the report lands there too.
    """
    say("===== FINAL REPORT: %d unique Cell ID(s) decoded =====" % len(found_cells))
    if not found_cells:
        say("  (no Cell IDs were decoded this run)")
        return
    for cell_id in sorted(found_cells):
        line = found_cells[cell_id]
        operator = operator_label(line)
        say("  " + line + (("  [%s]" % operator) if operator else ""))
    say("===== %d Cell ID(s): %s =====" % (
        len(found_cells),
        ", ".join(str(c) for c in sorted(found_cells)),
    ))


def take_lines(pending, chunk):
    """Split a byte stream on newlines. Return complete text lines and the remainder."""
    pending += chunk.replace(b"\r", b"\n")
    lines = []
    while b"\n" in pending:
        raw, pending = pending.split(b"\n", 1)
        text = raw.decode("utf-8", "replace")
        text = re.sub(r"\x1b\[[0-9;]*[A-Za-z]", "", text).strip()
        if text:
            lines.append(text)
    return lines, pending


def read_group(earfcn, hz, gain, group, known, dwell, find_seconds):
    """Read one cell. group is a sync group 0/1/2, or None to lock the strongest.

    find_seconds is how long to hunt for a tower before giving up (must exceed the
    Pluto's ~5 s open/calibrate plus sync time). Once a cell is found the reader is
    kept for `dwell` seconds so the SIB4/SIB5 neighbour lists are caught.
    Returns {cell_id, pci, line, neigh_earfcns, neigh_pcis, scheduled}.
    """
    tag = "group %d" % group if group is not None else "scan"
    say("%s  listening" % tag)
    command = [
        "stdbuf", "-oL", "-eL",
        _PDSCH,
        "-I", "soapy",
        "-a", _soapy_device(),
        "-f", str(hz),
        "-g", str(gain),
        "-Q",
        "-F",
        "-d",
    ]
    if group is not None:
        command += ["-l", str(group)]
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        bufsize=0,
    )
    streams = [process.stdout, process.stderr]
    pending = {process.stdout: b"", process.stderr: b""}
    started = time.monotonic()
    deadline = started + find_seconds
    limit = started + find_seconds + dwell + 20
    last_note = started
    saw_pci = False
    mib_seen = False
    pci_found_at = None
    mib_seen_at = None
    pci_num = None
    cell_id = None
    cell_line = None
    neigh_earfcns = set()
    neigh_pcis = set()
    scheduled = set()
    try:
        while time.monotonic() < deadline:
            if time.monotonic() - last_note >= 5:
                left = int(deadline - time.monotonic())
                say("%s  still listening, %d s left" % (tag, max(left, 0)))
                last_note = time.monotonic()
            # A real cell decodes its MIB within a second or two of being found,
            # even at a low PSR (PCI 430 did at PSR 2.2). A sync that never
            # decodes MIB is noise/too-weak and will not yield an ID, so give up
            # on it early instead of holding the group for the full dwell.
            if (_WEAK_TIMEOUT > 0 and saw_pci and not mib_seen and cell_id is None
                    and pci_found_at is not None
                    and time.monotonic() - pci_found_at >= _WEAK_TIMEOUT):
                say("%s  no MIB %.0f s after sync, giving up (too weak to read)" % (tag, _WEAK_TIMEOUT))
                break
            # MIB decoded but SIB1 (the Cell ID) has not arrived. At a usable
            # SNR it comes within ~1-2 s; if it has not by now it will not, so
            # stop instead of holding the full dwell.
            if (_SIB1_TIMEOUT > 0 and mib_seen and cell_id is None
                    and mib_seen_at is not None
                    and time.monotonic() - mib_seen_at >= _SIB1_TIMEOUT):
                say("%s  MIB but no SIB1 %.0f s on, giving up (too weak for the cell ID)" % (tag, _SIB1_TIMEOUT))
                break
            remaining = max(0, deadline - time.monotonic())
            ready, _, _ = select.select(streams, [], [], min(0.5, remaining))
            if not ready:
                if process.poll() is not None:
                    break
                continue
            for stream in ready:
                chunk = stream.read(4096)
                if not chunk:
                    if stream in streams:
                        streams.remove(stream)
                    continue
                lines, pending[stream] = take_lines(pending[stream], chunk)
                for text in lines:
                    if _MISS.search(text):
                        continue
                    pci = _PCI.search(text)
                    if pci and float(pci.group(3)) > 0:
                        pci_num = int(pci.group(1))
                        say("%s  PCI %s %s  PSR %s" % (tag, pci.group(1), pci.group(2), pci.group(3)))
                        last_note = time.monotonic()
                        if not saw_pci:
                            deadline = min(limit, time.monotonic() + dwell)
                            say("%s  tower found, reading for up to %d s" % (tag, dwell))
                            pci_found_at = time.monotonic()
                        saw_pci = True
                    if _RATE.search(text):
                        say("%s  sample rate %s MHz" % (tag, _RATE.search(text).group(1)))
                    if _MIB.search(text):
                        say("%s  MIB decoded, reading SIBs" % tag)
                        last_note = time.monotonic()
                        saw_pci = True
                        if not mib_seen:
                            mib_seen_at = time.monotonic()
                        mib_seen = True
                        deadline = min(limit, time.monotonic() + dwell)
                    found = _IDENTITY.search(text)
                    if found and cell_id is None:
                        if pci_num is None and process.stdout in streams:
                            ready_out, _, _ = select.select([process.stdout], [], [], 0.4)
                            if ready_out:
                                extra = process.stdout.read(4096)
                                if extra:
                                    extra_lines, pending[process.stdout] = take_lines(pending[process.stdout], extra)
                                    for extra_text in extra_lines:
                                        late = _PCI.search(extra_text)
                                        if late and float(late.group(3)) > 0:
                                            pci_num = int(late.group(1))
                        cell_id = int(found.group(1))
                        pci_text = ("PCI %d  " % pci_num) if pci_num is not None else ""
                        cell_line = (
                            "%.1f MHz (EARFCN %d)  %s28-bit %s  (0x%07X)  eNB %s  sector %s  TAC %s  PLMN %s"
                            % (
                                hz / 1e6,
                                earfcn,
                                pci_text,
                                found.group(1),
                                cell_id,
                                found.group(2),
                                found.group(3),
                                found.group(4),
                                found.group(5),
                            )
                        )
                        shown = "%s  %s" % (tag, cell_line)
                        if known:
                            shown += "  MATCH" if cell_id in known else "  not a known cell"
                        say(shown)
                        if dwell > 0:
                            say("%s  got ID, reading neighbour lists for up to %d s" % (tag, dwell))
                        last_note = time.monotonic()
                        saw_pci = True
                        deadline = min(limit, time.monotonic() + dwell)
                    sched = _SCHED.search(text)
                    if sched and not scheduled:
                        scheduled = set(int(x) for x in re.findall(r"SIB(\d+)", sched.group(1)))
                        say("%s  SIB1 schedule: %s" % (tag, " ".join("SIB%d" % s for s in sorted(scheduled))))
                        last_note = time.monotonic()
                    sib4 = _SIB4.search(text)
                    if sib4:
                        pcis = [int(x) for x in sib4.group(1).split() if x.isdigit()]
                        if pcis:
                            neigh_pcis.update(pcis)
                            say("%s  SIB4 co-channel PCIs: %s" % (tag, " ".join(str(p) for p in pcis)))
                            last_note = time.monotonic()
                    sib5 = _SIB5.search(text)
                    if sib5:
                        neighbour = int(sib5.group(1))
                        neigh_earfcns.add(neighbour)
                        pcis = [int(x) for x in (sib5.group(2) or "").split() if x.isdigit()]
                        neigh_pcis.update(pcis)
                        say("%s  SIB5 neighbour EARFCN %d (%s)" % (tag, neighbour, mhz_label(neighbour)))
                        last_note = time.monotonic()
            if cell_id is not None and (dwell == 0 or neigh_earfcns):
                break
            if not streams:
                break
        if cell_id is None:
            if saw_pci:
                say("%s  PCI %s present but SIB1 did not decode (too weak to read the ID)" % (tag, pci_num))
            else:
                say("%s  no cell" % tag)
        else:
            if 5 in scheduled and not neigh_earfcns:
                say("%s  cell schedules SIB5 but it did not decode - weak signal/PDCCH miss" % tag)
            elif scheduled and 5 not in scheduled:
                say("%s  this cell broadcasts no SIB5 (no inter-frequency neighbour list)" % tag)
        return {
            "cell_id": cell_id,
            "pci": pci_num,
            "line": cell_line,
            "neigh_earfcns": neigh_earfcns,
            "neigh_pcis": neigh_pcis,
            "scheduled": scheduled,
        }
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        for stream in (process.stdout, process.stderr):
            if stream:
                stream.close()


def read_channel_all(earfcn, hz, gain, known, deshadow=None):
    """One pdsch_ue launch with the scan build: it opens the radio once and
    decodes every sync group (0,1,2) in that one session, printing a Cell ID
    per group it can read. We read until the process exits and collect them all.

    deshadow overrides the global for this call (the smart sweep runs a fast
    pass with deshadow=False, then digs a channel with deshadow=True).

    Returns (neigh_earfcns, cells, present_pcis). The scan build stops each
    group at its SIB1, so no SIB4/SIB5 neighbour lists are captured here.
    """
    use_deshadow = _DESHADOW if deshadow is None else deshadow
    say("scan  one radio session, all sync groups")
    command = [
        "stdbuf", "-oL", "-eL",
        _PDSCH,
        "-I", "soapy",
        "-a", _soapy_device(),
        "-f", str(hz),
        "-g", str(gain),
        "-Q",
        "-F",
        "-d",
    ]
    env = dict(os.environ, DESHADOW=str(_DESHADOW_REPS)) if use_deshadow else None
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0, env=env)
    streams = [process.stdout, process.stderr]
    pending = {process.stdout: b"", process.stderr: b""}
    started = time.monotonic()
    # ultimate safety net; the binary normally exits on its own. De-shadow repeats
    # the whole scan several times, so allow much longer.
    hard_limit = started + (600 if use_deshadow else 120)
    # Two-phase stall guard. Opening the radio can take a while with no output, so
    # be patient until the first line. Once output is flowing, a total freeze means
    # the Lime has hung mid-decode (classic: retuning to the 23.04 MHz rate a false
    # 15 MHz MIB asked for) - kill it quickly instead of waiting the old 30 s.
    stall_open = 40   # grace before any output at all
    stall_run = 12    # freeze tolerance once the process is talking
    seen_output = False
    last_output = started
    cells = []
    seen = set()
    present = set()
    neigh_earfcns = set()
    neigh_pcis = set()
    pci_num = None
    try:
        while time.monotonic() < hard_limit:
            stall_secs = stall_run if seen_output else stall_open
            if time.monotonic() - last_output > stall_secs:
                say("scan  no output for %d s - radio stalled, moving on" % stall_secs)
                break
            ready, _, _ = select.select(streams, [], [], 0.5)
            if not ready:
                if process.poll() is not None:
                    break
                continue
            drained = False
            for stream in ready:
                chunk = stream.read(4096)
                if not chunk:
                    if stream in streams:
                        streams.remove(stream)
                    drained = True
                    continue
                last_output = time.monotonic()
                seen_output = True
                lines, pending[stream] = take_lines(pending[stream], chunk)
                for text in lines:
                    dp = re.search(r"DE-SHADOW pass (\d+)/(\d+)", text)
                    if dp:
                        say("--- de-shadow pass %s/%s ---" % (dp.group(1), dp.group(2)))
                        continue
                    grp = _GROUP.search(text)
                    if grp:
                        say("group %s  searching" % grp.group(1))
                        continue
                    if _MISS.search(text):
                        continue
                    pci = _PCI.search(text)
                    if pci and float(pci.group(3)) > 0:
                        pci_num = int(pci.group(1))
                        psr = float(pci.group(3))
                        # Only a strong peak counts as a cell heard; weak ones are
                        # shown but ignored so noise can't trigger a de-shadow dig.
                        if psr >= _PRESENT_FLOOR:
                            present.add(pci_num)
                        say("  PCI %s %s  PSR %s%s" % (pci.group(1), pci.group(2), pci.group(3),
                            "  (weak, ignored)" if psr < _PRESENT_FLOOR else ""))
                    if _RATE.search(text):
                        say("  sample rate %s MHz" % _RATE.search(text).group(1))
                    if _MIB.search(text):
                        say("  MIB decoded")
                    found = _IDENTITY.search(text)
                    if found:
                        cell_id = int(found.group(1))
                        if pci_num is not None:
                            present.add(pci_num)  # a decoded cell always counts as heard
                        pci_text = ("PCI %d  " % pci_num) if pci_num is not None else ""
                        cell_line = (
                            "%.1f MHz (EARFCN %d)  %s28-bit %s  (0x%07X)  eNB %s  sector %s  TAC %s  PLMN %s"
                            % (hz / 1e6, earfcn, pci_text, found.group(1), cell_id,
                               found.group(2), found.group(3), found.group(4), found.group(5))
                        )
                        if cell_id not in seen:
                            seen.add(cell_id)
                            cells.append((cell_id, cell_line))
                            shown = "  " + cell_line
                            if known:
                                shown += "  MATCH" if cell_id in known else ""
                            say(shown)
            if not streams or (drained and process.poll() is not None):
                break
    except KeyboardInterrupt:
        # Keep (don't lose) the cells already decoded this channel, e.g. during
        # a long de-shadow run, and tell the main loop to stop after.
        global _STOP
        _STOP = True
        say("  stopped - keeping the %d cell(s) decoded so far" % len(cells))
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        for stream in (process.stdout, process.stderr):
            if stream:
                stream.close()
    return neigh_earfcns, cells, present


def shadow_check_channel(earfcn, hz, gain):
    """Launch the scan build with SCAN_MULTI=1 and read the three 'MULTI group N
    distinct cells' lines, then stop the process. Returns {group: [pci, ...]}.

    More than one PCI in a group means two cells share that sync group -- the
    case the strongest-only search throws away (shadowing).
    """
    env = dict(os.environ, SCAN_MULTI="1")
    command = [
        "stdbuf", "-oL", "-eL",
        _PDSCH,
        "-I", "soapy",
        "-a", _soapy_device(),
        "-f", str(hz),
        "-g", str(gain),
        "-Q",
        "-F",
        "-d",
    ]
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0, env=env)
    streams = [process.stdout, process.stderr]
    pending = {process.stdout: b"", process.stderr: b""}
    groups = {}
    deadline = time.monotonic() + 60
    try:
        while time.monotonic() < deadline and len(groups) < 3:
            ready, _, _ = select.select(streams, [], [], 0.5)
            if not ready:
                if process.poll() is not None:
                    break
                continue
            for stream in ready:
                chunk = stream.read(4096)
                if not chunk:
                    if stream in streams:
                        streams.remove(stream)
                    continue
                lines, pending[stream] = take_lines(pending[stream], chunk)
                for text in lines:
                    m = _SHADOW.search(text)
                    if m:
                        g = int(m.group(1))
                        pcis = [int(p) for p in re.findall(r"PCI=(\d+)", m.group(2))]
                        groups[g] = pcis
                        say("  group %d: %s" % (g, ", ".join("PCI %d" % p for p in pcis) if pcis else "(none)"))
            if not streams:
                break
        return groups
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        for stream in (process.stdout, process.stderr):
            if stream:
                stream.close()


def run_shadow_check(seeds, gain):
    """Sweep the seed channels and report any sync group that holds >1 cell.

    This is the go/no-go for building full de-shadowing: if no group anywhere
    holds more than one cell, there is nothing for de-shadowing to recover.
    """
    say("SHADOW CHECK: scanning for sync groups that hold more than one cell")
    say("(needs the scan build of pdsch_ue; this is slower than a normal sweep)")
    hits = []
    for earfcn in seeds:
        hz = downlink_hz(earfcn)
        if hz is None:
            continue
        say("=== EARFCN %d (%.1f MHz) ===" % (earfcn, hz / 1e6))
        groups = shadow_check_channel(earfcn, hz, gain)
        for g, pcis in sorted(groups.items()):
            if len(pcis) > 1:
                hits.append((earfcn, hz, g, pcis))
                say("  *** group %d holds %d cells (%s) - SHADOWING"
                    % (g, len(pcis), ", ".join("PCI %d" % p for p in pcis)))
    say("")
    if hits:
        say("===== SHADOW CHECK: %d group(s) hold >1 cell - de-shadowing WOULD help here =====" % len(hits))
        for earfcn, hz, g, pcis in hits:
            say("  %.1f MHz (EARFCN %d) group %d: %s"
                % (hz / 1e6, earfcn, g, ", ".join("PCI %d" % p for p in pcis)))
    else:
        say("===== SHADOW CHECK: no group held more than one cell - no shadowing to recover here =====")


def run_deshadow_sweep(seeds, gains, known):
    """Smart de-shadow sweep: fast-scan every channel, and de-shadow only the
    channels where more cells were heard than decoded (a cell is hiding). Empty,
    noise, and fully-decoded channels are passed over, so the slow repeat work
    only lands where it pays off.

    gains is a list. With more than one gain, every channel's fast pass is read
    at each gain and the cells merged, so a strong cell that overloads at high
    gain and a weak cell that needs it are both caught; the dig then runs at the
    high gain, which hears the most. A single gain behaves as before.
    """
    multi_gain = len(gains) > 1
    if multi_gain:
        say("de-shadow sweep: fast scan every channel at gains %s, dig only where cells are hidden"
            % ", ".join("%g" % g for g in gains))
    else:
        say("de-shadow sweep: fast scan every channel, dig only where cells are hidden")
    dig_gain = max(gains)
    found = {}
    for earfcn in seeds:
        if _STOP:
            break
        hz = downlink_hz(earfcn)
        if hz is None:
            continue
        say("=== EARFCN %d (%.1f MHz) ===" % (earfcn, hz / 1e6))
        present = set()
        decoded = set()
        for gain in gains:
            if _STOP:
                break
            if multi_gain:
                say("gain %g" % gain)
            _neigh, cells, pres = read_channel_all(earfcn, hz, gain, known, deshadow=False)
            present |= pres
            for cell_id, line in cells:
                if cell_id not in found:
                    found[cell_id] = line
                decoded.add(cell_id)
        say("EARFCN %d: %d PCI(s) heard, %d Cell ID(s) decoded" % (earfcn, len(present), len(decoded)))
        # Dig whenever a PCI was heard that we did not get a Cell ID for. That
        # covers both a hidden co-channel cell (2 heard, 1 decoded) and a weak
        # cell whose PSS came through but whose SIB1 missed on the quick pass
        # (1 heard, 0 decoded) - both just need more dwelling passes.
        if len(present) > len(decoded):
            say("EARFCN %d: %d heard but only %d decoded - de-shadowing this channel%s"
                % (earfcn, len(present), len(decoded),
                   (" at gain %g" % dig_gain) if multi_gain else ""))
            quiet = 0
            dig_deadline = time.monotonic() + _DESHADOW_MAX_SECONDS
            while quiet < 2 and not _STOP:
                if time.monotonic() > dig_deadline:
                    say("  de-shadow time cap (%ds) reached for this channel - moving on"
                        % _DESHADOW_MAX_SECONDS)
                    break
                _n, more, _p = read_channel_all(earfcn, hz, dig_gain, known, deshadow=True)
                new = 0
                for cell_id, line in more:
                    if cell_id not in found:
                        found[cell_id] = line
                        new += 1
                if new:
                    say("  +%d new cell(s) from de-shadow" % new)
                    quiet = 0
                else:
                    quiet += 1
                    say("  de-shadow lap added nothing new (%d/2)" % quiet)
    say("")
    final_report(found)


def scan_channel(earfcn, gain, groups, known, dwell, find_seconds, single):
    """Scan one EARFCN.

    single=True   one strongest-cell run (fast, one Cell ID per channel).
    single=False  a quick presence check first (one strongest-cell run, no dwell);
                  if nothing syncs the channel is skipped, otherwise all sync
                  groups are decoded so weak co-channel cells are caught too.

    Return (neigh_earfcns, cells, present_pcis) where cells is a list of
    (cell_id, line) and present_pcis is every PCI heard, decoded or not.
    """
    hz = downlink_hz(earfcn)
    if hz is None:
        say("EARFCN %d is outside the band table, skipping" % earfcn)
        return set(), [], set()
    say("=== EARFCN %d (%.1f MHz) ===" % (earfcn, hz / 1e6))
    if _SCAN_ALL:
        neigh_earfcns, cells, present = read_channel_all(earfcn, hz, gain, known)
        decoded = {cid for cid, _line in cells}
        if present:
            say("EARFCN %d summary: %d cell(s) heard (PCI %s), %d Cell ID(s) decoded"
                % (earfcn, len(present), " ".join(str(p) for p in sorted(present)), len(decoded)))
        else:
            say("EARFCN %d  nothing here" % earfcn)
        return neigh_earfcns, cells, present
    neigh_earfcns = set()
    cells = []
    present = set()
    if single:
        runs = [None]
    else:
        triage = read_group(earfcn, hz, gain, None, known, 0, find_seconds)
        if triage["pci"] is None and triage["cell_id"] is None:
            say("EARFCN %d  nothing here, skipping the groups" % earfcn)
            return set(), [], set()
        if triage["pci"] is not None:
            present.add(triage["pci"])
        neigh_earfcns.update(triage["neigh_earfcns"])
        skip_group = None
        if triage["cell_id"] is not None:
            cells.append((triage["cell_id"], triage["line"]))
            if triage["pci"] is not None:
                skip_group = triage["pci"] % 3
            say("EARFCN %d  strongest cell decoded; checking the other sync groups" % earfcn)
        else:
            say("EARFCN %d  cell present, decoding all sync groups" % earfcn)
        runs = [g for g in groups if g != skip_group]
    for group in runs:
        result = read_group(earfcn, hz, gain, group, known, dwell, find_seconds)
        if result["pci"] is not None:
            present.add(result["pci"])
        if result["cell_id"] is not None:
            cells.append((result["cell_id"], result["line"]))
        neigh_earfcns.update(result["neigh_earfcns"])
    decoded = {cid for cid, _line in cells}
    if present:
        say(
            "EARFCN %d summary: %d cell(s) heard (PCI %s), %d Cell ID(s) decoded"
            % (earfcn, len(present), " ".join(str(p) for p in sorted(present)), len(decoded))
        )
    return neigh_earfcns, cells, present


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Read cells on an EARFCN, follow SIB5 neighbours, or sweep a band plan."
    )
    seed = parser.add_mutually_exclusive_group(required=True)
    seed.add_argument("--earfcn", type=int, help="seed LTE downlink channel number")
    seed.add_argument("--sweep", action="store_true", help="seed from the built-in UK band plan")
    parser.add_argument("--gain", type=float, default=70.0, help="RX gain passed to pdsch_ue")
    parser.add_argument("--driver", default="plutosdr", help="SoapySDR driver: plutosdr (default) or lime")
    parser.add_argument("--device-args", default="", help="extra Soapy device args, comma-separated, e.g. 'rxant=LNAL' to force the Lime low-band port")
    parser.add_argument("--weak-timeout", type=float, default=5.0, help="give up on a sync group this many seconds after it finds a tower if no MIB has decoded (default 5; 0 = wait the full dwell, old behaviour). MIB, not PSR, is the real gate -- a weak real cell still decodes MIB fast.")
    parser.add_argument("--sib1-timeout", type=float, default=8.0, help="give up this many seconds after MIB decodes if the Cell ID (SIB1) has not (default 8; 0 = wait the full dwell). Catches the overload/too-weak case where MIB reads but SIB1 never will.")
    parser.add_argument("--gain-sweep", default="", help="read each channel at several gains and merge the cells, e.g. '45,68'. Catches both strong cells that overload at high gain and weak cells that need it. Runs full mode (disables --fast); each cell in the report is tagged with the gain that decoded it.")
    parser.add_argument("--scan-all", action="store_true", help="one pdsch_ue launch per channel that decodes all 3 sync groups in a single open radio session (needs the osprey scan build of pdsch_ue). Faster: one radio open per channel instead of 3-4.")
    parser.add_argument("--shadow-check", action="store_true", help="diagnostic (needs the scan build): sweep and report any sync group holding more than one cell. Use it in a dense/city location to decide whether full de-shadowing is worth building. Does not decode Cell IDs.")
    parser.add_argument("--deshadow", action="store_true", help="repeat each channel's scan several times so cells sharing a sync group (shadowed) are decoded too, not just the strongest. Slower; needs --scan-all and the scan build. Best on a single --earfcn a shadow-check flagged.")
    parser.add_argument("--deshadow-reps", type=int, default=10, help="how many times --deshadow repeats each channel's scan (default 10; more = better odds of catching an interfering shadowed cell, but slower)")
    parser.add_argument("--triage", action="store_true", help="fast PSS check before each channel; skip channels with no cell (faster sweeps, off by default)")
    parser.add_argument("--triage-threshold", type=float, default=6.0, help="PSS peak/RMS ratio to count a channel as occupied (default 6.0); 0 = measure and print only, never skip (for calibration)")
    parser.add_argument(
        "--cell-id",
        type=int,
        action="append",
        default=[],
        help="known 28-bit cell identity. Repeat for more. A match is marked MATCH",
    )
    parser.add_argument("--group", type=int, nargs="+", default=[0, 1, 2], help="sync groups to search (normal mode)")
    parser.add_argument("--find", type=int, default=None, help="seconds to hunt for a tower (default 20, fast 18)")
    parser.add_argument("--dwell", type=int, default=None, help="seconds to read a found cell for SIB4/SIB5 (default 30, fast 8)")
    parser.add_argument("--fast", action="store_true", help="one strongest-cell run per channel, short timers, no crawl")
    parser.add_argument("--no-crawl", action="store_true", help="do not follow SIB5 neighbour frequencies")
    parser.add_argument("--once", action="store_true", help="stop after one full crawl/sweep instead of looping")
    parser.add_argument("--log", help="append every Cell ID found to this file")
    args = parser.parse_args(argv)

    global _DRIVER, _DEVICE_ARGS, _TRIAGE, _TRIAGE_THRESHOLD, _LOG, _WEAK_TIMEOUT, _SIB1_TIMEOUT, _SCAN_ALL
    _DRIVER = args.driver
    _DEVICE_ARGS = args.device_args or ""
    _TRIAGE = args.triage
    _TRIAGE_THRESHOLD = args.triage_threshold
    _LOG = args.log
    _WEAK_TIMEOUT = args.weak_timeout
    _SIB1_TIMEOUT = args.sib1_timeout
    _SCAN_ALL = args.scan_all
    global _DESHADOW, _DESHADOW_REPS
    _DESHADOW = args.deshadow
    _DESHADOW_REPS = max(2, args.deshadow_reps)
    if _DESHADOW and not _SCAN_ALL:
        _SCAN_ALL = True  # de-shadow lives in the scan build's one-launch path
    if _SCAN_ALL:
        say("scan-all: one radio session per channel (needs the scan build of pdsch_ue)")
    if _DESHADOW:
        say("de-shadow: repeating each channel's scan to catch cells sharing a sync group")
    say("radio: driver=%s%s" % (_DRIVER, (" " + _DEVICE_ARGS) if _DEVICE_ARGS else ""))
    if _TRIAGE:
        if _TRIAGE_THRESHOLD <= 0:
            say("triage: PSS measure-only (never skips) - calibration run")
        else:
            say("triage: fast PSS gate on, skip below ratio %.1f" % _TRIAGE_THRESHOLD)

    groups = []
    for group in args.group:
        if group not in (0, 1, 2):
            sys.exit("group must be 0, 1 or 2")
        if group not in groups:
            groups.append(group)

    gains = parse_gains(args.gain_sweep) or [args.gain]
    multi_gain = len(gains) > 1
    if multi_gain and args.fast:
        say("gain-sweep runs full (non-fast) mode; ignoring --fast")
    single = args.fast and not multi_gain
    find_seconds = args.find if args.find is not None else 18
    dwell = args.dwell if args.dwell is not None else (8 if single else 15)
    crawl = (not args.no_crawl) and (not single)
    triage_gain = max(gains)

    if not os.path.isfile(_PDSCH) or not os.access(_PDSCH, os.X_OK):
        sys.exit("pdsch_ue was not found at %s" % _PDSCH)

    if args.sweep:
        seeds = [e for e in _SWEEP_PLAN if downlink_hz(e) is not None]
        say("sweep: %d seed channels from the UK band plan" % len(seeds))
    else:
        if downlink_hz(args.earfcn) is None:
            sys.exit("EARFCN %d is outside the band table in this launcher." % args.earfcn)
        seeds = [args.earfcn]
        say("seed EARFCN %d (%s)" % (args.earfcn, mhz_label(args.earfcn)))

    say(
        "%s; find %d s; dwell %d s; crawl %s. Ctrl+C stops."
        % (
            "fast: one strongest-cell run per channel"
            if single
            else ("groups %s, empty channels skipped after a quick check" % " ".join(str(g) for g in groups)),
            find_seconds,
            dwell,
            "off" if not crawl else "on",
        )
    )
    if _WEAK_TIMEOUT > 0:
        say("weak-group timeout: drop a group %g s after sync if no MIB decodes" % _WEAK_TIMEOUT)
    if _SIB1_TIMEOUT > 0:
        say("SIB1 timeout: drop a group %g s after MIB if no Cell ID decodes" % _SIB1_TIMEOUT)
    if multi_gain:
        say("gain-sweep: reading every channel at gains %s and merging" % ", ".join("%g" % g for g in gains))
    if args.cell_id:
        say("known cells: %s" % ", ".join(str(cell) for cell in args.cell_id))
    if args.log:
        say("logging the full run to %s" % args.log)
    known = set(args.cell_id)

    if args.shadow_check:
        try:
            run_shadow_check(seeds, args.gain)
        except KeyboardInterrupt:
            say("shadow check stopped")
        return 0

    if args.sweep and _DESHADOW:
        try:
            run_deshadow_sweep(seeds, gains, known)
        except KeyboardInterrupt:
            say("de-shadow sweep stopped")
        return 0

    found_cells = {}
    quiet_laps = 0
    try:
        while True:
            lap_start_count = len(found_cells)
            queue = list(seeds)
            scanned = set()
            while queue:
                earfcn = queue.pop(0)
                if earfcn in scanned:
                    continue
                scanned.add(earfcn)
                hz = downlink_hz(earfcn)
                if hz is None:
                    say("EARFCN %d is outside the band table, skipping" % earfcn)
                    continue
                # One fast PSS triage per channel (at the most sensitive gain).
                # If it is confidently empty, skip every gain pass on it.
                if _TRIAGE and not _triage_channel(hz, triage_gain, earfcn):
                    say("done %s: triage skip, %d cells total, %d channels queued" % (earfcn, len(found_cells), len(queue)))
                    continue
                neigh_earfcns = set()
                for gain in gains:
                    if multi_gain:
                        say("--- EARFCN %d at gain %g ---" % (earfcn, gain))
                    g_neigh, cells, _present = scan_channel(earfcn, gain, groups, known, dwell, find_seconds, single)
                    neigh_earfcns.update(g_neigh)
                    for cell_id, line in cells:
                        if cell_id not in found_cells:
                            found_cells[cell_id] = line + ("  (gain %g)" % gain if multi_gain else "")
                    if _STOP:
                        break
                if _STOP:
                    break
                if crawl:
                    for neighbour in sorted(neigh_earfcns):
                        if neighbour in scanned or neighbour in queue:
                            continue
                        if downlink_hz(neighbour) is not None:
                            queue.append(neighbour)
                            say("  queued neighbour EARFCN %d (%s)" % (neighbour, mhz_label(neighbour)))
                        else:
                            say("  neighbour EARFCN %d is %s (not queued)" % (neighbour, mhz_label(neighbour)))
                say("done %s: %d cells total, %d channels queued" % (earfcn, len(found_cells), len(queue)))
            say("pass complete: %d unique cells across %d channels" % (len(found_cells), len(scanned)))
            if found_cells:
                say("cells found: %s" % ", ".join(str(cell) for cell in sorted(found_cells)))
            if args.once or _STOP:
                break
            # De-shadow auto-stop: once a whole lap adds no new cell (twice in a
            # row, to ride out a probabilistic miss), it has caught what it can.
            if _DESHADOW and found_cells:
                if len(found_cells) == lap_start_count:
                    quiet_laps += 1
                    say("de-shadow: no new cell this lap (%d/2 quiet laps)" % quiet_laps)
                    if quiet_laps >= 2:
                        say("de-shadow: converged - nothing new in 2 laps, stopping")
                        break
                else:
                    quiet_laps = 0
            say("looping again (Ctrl+C to stop)")
    except KeyboardInterrupt:
        say("stopped: %d unique cells found" % len(found_cells))
        final_report(found_cells)
        return 0
    final_report(found_cells)
    return 0


if __name__ == "__main__":
    sys.exit(main())
