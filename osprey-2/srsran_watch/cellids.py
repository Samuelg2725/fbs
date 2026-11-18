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


def downlink_hz(earfcn):
    """Hz of one downlink EARFCN, or None when the table above does not cover it."""
    for _band, start, offset, first, last in _BANDS:
        if first <= earfcn <= last:
            return (start + (earfcn - offset)) * 100_000
    return None


def mhz_label(earfcn):
    hz = downlink_hz(earfcn)
    return "%.1f MHz" % (hz / 1e6) if hz is not None else "outside band table"


def say(text):
    print(datetime.now().strftime("%Y-%m-%d %H:%M:%S  ") + text, flush=True)


def log_cell(path, text):
    if not path:
        return
    try:
        with open(path, "a", encoding="utf-8") as handle:
            handle.write(datetime.now().strftime("%Y-%m-%d %H:%M:%S  ") + text + "\n")
    except OSError as exc:
        say("could not write log %s: %s" % (path, exc))


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
        "-a", "driver=plutosdr",
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
                        saw_pci = True
                    if _RATE.search(text):
                        say("%s  sample rate %s MHz" % (tag, _RATE.search(text).group(1)))
                    if _MIB.search(text):
                        say("%s  MIB decoded, reading SIBs" % tag)
                        last_note = time.monotonic()
                        saw_pci = True
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


def scan_channel(earfcn, gain, groups, known, dwell, find_seconds, single):
    """Scan one EARFCN. single=True does one strongest-cell run; else one per group.

    Return (neigh_earfcns, cells, present_pcis) where cells is a list of
    (cell_id, line) and present_pcis is every PCI heard, decoded or not.
    """
    hz = downlink_hz(earfcn)
    if hz is None:
        say("EARFCN %d is outside the band table, skipping" % earfcn)
        return set(), [], set()
    say("=== EARFCN %d (%.1f MHz) ===" % (earfcn, hz / 1e6))
    neigh_earfcns = set()
    cells = []
    present = set()
    runs = [None] if single else list(groups)
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
    parser.add_argument("--gain", type=float, default=70.0, help="Pluto gain passed to pdsch_ue")
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

    groups = []
    for group in args.group:
        if group not in (0, 1, 2):
            sys.exit("group must be 0, 1 or 2")
        if group not in groups:
            groups.append(group)

    find_seconds = args.find if args.find is not None else (18 if args.fast else 20)
    dwell = args.dwell if args.dwell is not None else (8 if args.fast else 30)
    crawl = (not args.no_crawl) and (not args.fast)
    single = args.fast

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
            "fast: one strongest-cell run per channel" if single else ("groups %s" % " ".join(str(g) for g in groups)),
            find_seconds,
            dwell,
            "off" if not crawl else "on",
        )
    )
    if args.cell_id:
        say("known cells: %s" % ", ".join(str(cell) for cell in args.cell_id))
    if args.log:
        say("logging Cell IDs to %s" % args.log)
    known = set(args.cell_id)

    found_cells = set()
    try:
        while True:
            queue = list(seeds)
            scanned = set()
            while queue:
                earfcn = queue.pop(0)
                if earfcn in scanned:
                    continue
                scanned.add(earfcn)
                neigh_earfcns, cells, _present = scan_channel(earfcn, args.gain, groups, known, dwell, find_seconds, single)
                for cell_id, line in cells:
                    if cell_id not in found_cells:
                        found_cells.add(cell_id)
                        log_cell(args.log, line)
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
            if args.once:
                break
            say("looping again (Ctrl+C to stop)")
    except KeyboardInterrupt:
        say("stopped: %d unique cells found" % len(found_cells))
        return 0
    return 0


if __name__ == "__main__":
    sys.exit(main())
