#!/usr/bin/env python3
"""Read the cells on an EARFCN and follow their neighbour lists.

This file does not decode. It starts the Pluto pdsch_ue once per sync group
(the last digit of the physical cell ID modulo 3), reads the strongest cell
in each group, and then keeps the reader running long enough to also catch
that cell's SIB4 and SIB5 neighbour lists:

  SIB4  physical cell IDs of other cells on the SAME frequency (co-channel)
  SIB5  the OTHER EARFCNs this cell lists as neighbours, and their PCIs

The SIB5 EARFCNs are the operator's local channel plan. By default the
scanner follows them: it queues each new neighbour frequency and scans it
too, so one seed EARFCN crawls outward across the whole local deployment.
Use --no-crawl to scan only the seed frequency.
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

FIND_SECONDS = 20  # time to find a tower before giving up on a frequency


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


def read_group(earfcn, hz, gain, group, known, dwell):
    """Read one sync group. Return {cell_id, pci, neigh_earfcns, neigh_pcis}.

    When a cell is found the reader is kept running for `dwell` seconds so the
    SIB4/SIB5 neighbour lists (sent in later SI windows than SIB1) are caught.
    """
    say("group %d  listening" % group)
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
        "-l", str(group),
    ]
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        bufsize=0,
    )
    streams = [process.stdout, process.stderr]
    pending = {process.stdout: b"", process.stderr: b""}
    started = time.monotonic()
    deadline = started + FIND_SECONDS
    limit = started + FIND_SECONDS + dwell + 20
    last_note = started
    saw_pci = False
    pci_num = None
    cell_id = None
    neigh_earfcns = set()
    neigh_pcis = set()
    scheduled = set()
    try:
        while time.monotonic() < deadline:
            if time.monotonic() - last_note >= 5:
                left = int(deadline - time.monotonic())
                say("group %d  still listening, %d s left" % (group, max(left, 0)))
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
                        say("group %d  PCI %s %s  PSR %s" % (group, pci.group(1), pci.group(2), pci.group(3)))
                        last_note = time.monotonic()
                        if not saw_pci:
                            deadline = min(limit, time.monotonic() + dwell)
                            say("group %d  tower found, collecting ID + neighbours for up to %d s" % (group, dwell))
                        saw_pci = True
                    if _RATE.search(text):
                        say("group %d  sample rate %s MHz" % (group, _RATE.search(text).group(1)))
                    if _MIB.search(text):
                        say("group %d  MIB decoded, reading SIBs" % group)
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
                        line = (
                            "group %d  %s28-bit %s  (0x%07X)  eNB %s  sector %s  TAC %s  PLMN %s"
                            % (
                                group,
                                pci_text,
                                found.group(1),
                                cell_id,
                                found.group(2),
                                found.group(3),
                                found.group(4),
                                found.group(5),
                            )
                        )
                        if known:
                            line += "  MATCH" if cell_id in known else "  not a known cell"
                        say(line)
                        say("group %d  got ID, reading neighbour lists (SIB4/SIB5) for up to %d s" % (group, dwell))
                        last_note = time.monotonic()
                        saw_pci = True
                        deadline = min(limit, time.monotonic() + dwell)
                    sched = _SCHED.search(text)
                    if sched and not scheduled:
                        scheduled = set(int(x) for x in re.findall(r"SIB(\d+)", sched.group(1)))
                        say("group %d  SIB1 schedule: %s" % (group, " ".join("SIB%d" % s for s in sorted(scheduled))))
                        last_note = time.monotonic()
                    sib4 = _SIB4.search(text)
                    if sib4:
                        pcis = [int(x) for x in sib4.group(1).split() if x.isdigit()]
                        if pcis:
                            neigh_pcis.update(pcis)
                            say("group %d  SIB4 co-channel PCIs: %s" % (group, " ".join(str(p) for p in pcis)))
                            last_note = time.monotonic()
                    sib5 = _SIB5.search(text)
                    if sib5:
                        neighbour = int(sib5.group(1))
                        neigh_earfcns.add(neighbour)
                        pcis = [int(x) for x in (sib5.group(2) or "").split() if x.isdigit()]
                        neigh_pcis.update(pcis)
                        say("group %d  SIB5 neighbour EARFCN %d (%s)" % (group, neighbour, mhz_label(neighbour)))
                        last_note = time.monotonic()
            if cell_id is not None and neigh_earfcns:
                say("group %d  got ID and neighbour frequencies, moving on" % group)
                break
            if not streams:
                break
        if cell_id is None:
            if saw_pci:
                say("group %d  saw a cell, SIB1 did not pass" % group)
            else:
                say("group %d  no cell" % group)
        else:
            if 5 in scheduled and not neigh_earfcns:
                say("group %d  cell schedules SIB5 but it did not decode - weak signal/PDCCH miss; try --gain 73 or a better antenna" % group)
            elif scheduled and 5 not in scheduled:
                say("group %d  this cell broadcasts no SIB5 (no inter-frequency neighbour list)" % group)
        return {
            "cell_id": cell_id,
            "pci": pci_num,
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


def scan_channel(earfcn, gain, groups, known, dwell):
    """Scan all requested groups on one EARFCN. Return the set of neighbour EARFCNs."""
    hz = downlink_hz(earfcn)
    if hz is None:
        say("EARFCN %d is outside the band table, skipping" % earfcn)
        return set(), set()
    say("=== EARFCN %d (%.1f MHz) ===" % (earfcn, hz / 1e6))
    neigh_earfcns = set()
    cells = set()
    for group in groups:
        result = read_group(earfcn, hz, gain, group, known, dwell)
        if result["cell_id"] is not None:
            cells.add(result["cell_id"])
        neigh_earfcns.update(result["neigh_earfcns"])
    return neigh_earfcns, cells


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Read the cells on an EARFCN and follow their SIB5 neighbour frequencies."
    )
    parser.add_argument("--earfcn", type=int, required=True, help="seed LTE downlink channel number")
    parser.add_argument("--gain", type=float, default=70.0, help="Pluto gain passed to pdsch_ue")
    parser.add_argument(
        "--cell-id",
        type=int,
        action="append",
        default=[],
        help="known 28-bit cell identity. Repeat for more than one. A match is marked MATCH",
    )
    parser.add_argument(
        "--group",
        type=int,
        nargs="+",
        default=[0, 1, 2],
        help="which groups to search. Example: --group 0 1   or   --group 2",
    )
    parser.add_argument(
        "--dwell",
        type=int,
        default=30,
        help="seconds to keep reading a found cell for its ID and SIB4/SIB5 neighbours",
    )
    parser.add_argument(
        "--no-crawl",
        action="store_true",
        help="scan only --earfcn; do not follow SIB5 neighbour frequencies",
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="stop after one full crawl instead of looping forever",
    )
    args = parser.parse_args(argv)
    groups = []
    for group in args.group:
        if group not in (0, 1, 2):
            sys.exit("group must be 0, 1 or 2")
        if group not in groups:
            groups.append(group)

    hz = downlink_hz(args.earfcn)
    if hz is None:
        sys.exit("EARFCN %d is outside the band table in this launcher." % args.earfcn)
    if not os.path.isfile(_PDSCH) or not os.access(_PDSCH, os.X_OK):
        sys.exit("pdsch_ue was not found at %s" % _PDSCH)

    say("seed EARFCN %d (%.1f MHz)" % (args.earfcn, hz / 1e6))
    say(
        "groups %s; dwell %d s; crawl %s. Ctrl+C stops."
        % (" ".join(str(g) for g in groups), args.dwell, "off" if args.no_crawl else "on")
    )
    if args.cell_id:
        say("known cells: %s" % ", ".join(str(cell) for cell in args.cell_id))
    known = set(args.cell_id)

    found_cells = set()
    try:
        while True:
            queue = [args.earfcn]
            scanned = set()
            while queue:
                earfcn = queue.pop(0)
                if earfcn in scanned:
                    continue
                scanned.add(earfcn)
                neigh_earfcns, cells = scan_channel(earfcn, args.gain, groups, known, args.dwell)
                found_cells.update(cells)
                if not args.no_crawl:
                    for neighbour in sorted(neigh_earfcns):
                        if neighbour in scanned or neighbour in queue:
                            continue
                        if downlink_hz(neighbour) is not None:
                            queue.append(neighbour)
                            say("  queued neighbour EARFCN %d (%s)" % (neighbour, mhz_label(neighbour)))
                        else:
                            say("  neighbour EARFCN %d is %s (not queued)" % (neighbour, mhz_label(neighbour)))
                say(
                    "EARFCN %d done: %d cells total, %d channels queued"
                    % (earfcn, len(found_cells), len(queue))
                )
            say("crawl pass complete: %d unique cells across %d channels" % (len(found_cells), len(scanned)))
            if found_cells:
                say("cells found: %s" % ", ".join(str(cell) for cell in sorted(found_cells)))
            if args.once:
                break
            say("looping the crawl again (Ctrl+C to stop)")
    except KeyboardInterrupt:
        say("stopped: %d unique cells found" % len(found_cells))
        return 0
    return 0


if __name__ == "__main__":
    sys.exit(main())
