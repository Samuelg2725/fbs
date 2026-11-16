#!/usr/bin/env python3
"""Read every cell srsRAN can separate on one EARFCN.

This file does not decode. It starts the Pluto pdsch_ue three times, once
for each sync group, then repeats. A sync group is the last digit of the
physical cell ID modulo 3. pdsch_ue returns the strongest cell in that
group, then this script reads its 28-bit identity and moves to the next
group. A known identity is marked MATCH.
"""

from __future__ import annotations

import argparse
import math
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


def read_group(earfcn, hz, gain, group, known, search_s=4.0, sib1_s=6.0, miss_limit=3,
               rf_args="driver=plutosdr"):
    """Read the strongest cell in one sync group. Return its 28-bit identity, or None.

    search_s   seconds to look for a tower before giving up on this group.
    sib1_s     seconds to wait for the cell ID once a tower is found.
    miss_limit give up early after pdsch_ue reports "no cell" this many times.
    rf_args     SoapySDR args for pdsch_ue, e.g. "driver=plutosdr,uri=ip:10.1.2.11".
    """
    say("group %d  listening" % group)
    command = [
        "stdbuf", "-oL", "-eL",
        _PDSCH,
        "-I", "soapy",
        "-a", rf_args,
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
    # search_s to find a tower. Once one is found, sib1_s for the cell ID.
    # A real cell locks in ~1-2 s and a decodable SIB1 arrives in ~1-3 s, so
    # these short windows lose almost nothing and the empty groups, which used
    # to burn the full 20 s each, now die in a couple of seconds.
    started = time.monotonic()
    deadline = started + search_s
    limit = started + search_s + sib1_s
    last_note = started
    note_every = min(5.0, max(1.0, search_s / 2.0))
    saw_pci = False
    pci_num = None
    pci_at = None
    id_at = None
    miss_streak = 0

    def result(cell=None):
        """Return dict with the cell plus the step timings calibration needs."""
        return {
            "group": group,
            "cell": cell,
            "pci": pci_num,
            "t_search": (pci_at - started) if pci_at is not None else None,
            "t_decode": (id_at - pci_at) if (pci_at is not None and id_at is not None) else None,
        }

    try:
        while time.monotonic() < deadline:
            if time.monotonic() - last_note >= note_every:
                left = int(deadline - time.monotonic())
                say("group %d  still searching, %d s left" % (group, max(left, 0)))
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
                        # pdsch_ue is telling us this group is empty. Before, we
                        # ignored it and waited out the whole window; now we bail
                        # after a few consecutive misses (unless a tower showed up).
                        if not saw_pci:
                            miss_streak += 1
                            if miss_streak >= miss_limit:
                                say("group %d  no cell (radio reported empty %dx)" % (group, miss_streak))
                                return result()
                        continue
                    pci = _PCI.search(text)
                    if pci and float(pci.group(3)) > 0:
                        pci_num = int(pci.group(1))
                        miss_streak = 0
                        say("group %d  PCI %s %s  PSR %s" % (group, pci.group(1), pci.group(2), pci.group(3)))
                        last_note = time.monotonic()
                        if not saw_pci:
                            pci_at = time.monotonic()
                            deadline = min(limit, time.monotonic() + sib1_s)
                            say("group %d  tower found at %.1f s, waiting up to %.0f s for the cell ID"
                                % (group, pci_at - started, sib1_s))
                        saw_pci = True
                    if _RATE.search(text):
                        say("group %d  sample rate %s MHz" % (group, _RATE.search(text).group(1)))
                    if _MIB.search(text):
                        say("group %d  MIB decoded, reading SIB1" % group)
                        last_note = time.monotonic()
                    found = _IDENTITY.search(text)
                    if found:
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
                        id_at = time.monotonic()
                        cell = int(found.group(1))
                        pci_text = ("PCI %d  " % pci_num) if pci_num is not None else ""
                        line = (
                            "group %d  %s28-bit %s  (0x%07X)  eNB %s  sector %s  TAC %s  PLMN %s"
                            % (
                                group,
                                pci_text,
                                found.group(1),
                                cell,
                                found.group(2),
                                found.group(3),
                                found.group(4),
                                found.group(5),
                            )
                        )
                        if known:
                            line += "  MATCH" if cell in known else "  not a known cell"
                        say(line)
                        return result(cell)
            if not streams:
                break
        if saw_pci:
            say("group %d  saw a cell, SIB1 did not pass" % group)
        else:
            say("group %d  no cell" % group)
        return result()
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


def _pct(values, frac):
    """Percentile on a copy of values. frac in [0, 1]."""
    if not values:
        return None
    s = sorted(values)
    idx = min(len(s) - 1, int(round(frac * (len(s) - 1))))
    return s[idx]


def _suggest(values, floor):
    """A timeout that covers the worst sample seen, with margin. Whole seconds."""
    if not values:
        return int(floor)
    return max(int(floor), int(math.ceil(max(values) * 1.3 + 1.0)))


def _report(searches, decodes):
    """Print the calibration summary and the suggested timeouts."""
    say("=================== CALIBRATION SUMMARY ===================")
    if not searches and not decodes:
        say("No cells found. Check antenna, --gain, --pluto-uri and the EARFCN,")
        say("then run calibration again. Current defaults left unchanged.")
        return
    if searches:
        say("search time  (group start -> tower found), %d sample(s):" % len(searches))
        say("   min %.1f   median %.1f   worst %.1f s"
            % (min(searches), _pct(searches, 0.5), max(searches)))
    if decodes:
        say("decode time  (tower found -> cell ID), %d sample(s):" % len(decodes))
        say("   min %.1f   median %.1f   worst %.1f s"
            % (min(decodes), _pct(decodes, 0.5), max(decodes)))
    else:
        say("towers were found but no SIB1 decoded in the window.")
        say("that is low SNR/gain, not a timeout problem - try --gain or the antenna.")
    rec_search = _suggest(searches, floor=3.0)
    rec_sib1 = _suggest(decodes, floor=5.0)
    say("---------------------------------------------------------")
    say("SUGGESTED:  --search-timeout %d  --sib1-timeout %d" % (rec_search, rec_sib1))
    say("(worst seen x1.3 + 1 s, so a slow-but-reachable cell is not cut off)")
    say("then run normally, e.g.:")
    say("   python3 cellids.py --earfcn <N> --search-timeout %d --sib1-timeout %d"
        % (rec_search, rec_sib1))


def calibrate(earfcn, hz, gain, groups, rf_args, passes, miss_limit):
    """Measure real search/decode times with generous windows, then recommend
    timeouts. Nothing is cut off while measuring, so the numbers are the truth
    about this radio, this spot, right now."""
    cal_search = 20.0
    cal_sib1 = 25.0
    say("CALIBRATION: %d pass(es), generous windows (%.0f s search, %.0f s SIB1)."
        % (passes, cal_search, cal_sib1))
    say("Measuring how long YOUR setup actually takes. Ctrl+C to stop and still get a summary.")
    searches = []
    decodes = []
    try:
        for p in range(passes):
            say("--- calibration pass %d/%d ---" % (p + 1, passes))
            for group in groups:
                r = read_group(
                    earfcn, hz, gain, group, set(),
                    search_s=cal_search, sib1_s=cal_sib1,
                    miss_limit=miss_limit, rf_args=rf_args,
                )
                if r["t_search"] is not None:
                    searches.append(r["t_search"])
                if r["t_decode"] is not None:
                    decodes.append(r["t_decode"])
    except KeyboardInterrupt:
        say("calibration stopped early")
    _report(searches, decodes)
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Read the cells on one EARFCN. This file does not decode."
    )
    parser.add_argument("--earfcn", type=int, required=True, help="LTE downlink channel number")
    parser.add_argument("--gain", type=float, default=70.0, help="Pluto gain passed to pdsch_ue")
    parser.add_argument(
        "--pluto-uri",
        default="ip:10.1.2.11",
        help="SoapySDR uri for the Pluto, e.g. ip:10.1.2.11 (Ethernet) or usb: . "
             "Pass an empty string to let Soapy auto-pick (may use USB).",
    )
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
        "--search-timeout",
        type=float,
        default=4.0,
        help="seconds to look for a tower in each group before giving up (was 20). Raise it to catch weaker cells.",
    )
    parser.add_argument(
        "--sib1-timeout",
        type=float,
        default=6.0,
        help="seconds to wait for the cell ID once a tower is found (was 15).",
    )
    parser.add_argument(
        "--miss-limit",
        type=int,
        default=3,
        help="give up on a group after pdsch_ue reports 'no cell' this many times. Raise it to be more patient.",
    )
    parser.add_argument(
        "--calibrate",
        action="store_true",
        help="measure how long search and decode really take here, print suggested timeouts, then exit.",
    )
    parser.add_argument(
        "--calibrate-passes",
        type=int,
        default=3,
        help="how many times to cycle the groups during calibration (default 3).",
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

    rf_args = "driver=plutosdr"
    if args.pluto_uri:
        rf_args += ",uri=%s" % args.pluto_uri

    say("%d Hz  EARFCN %d" % (hz, args.earfcn))
    say("radio: %s" % rf_args)

    if args.calibrate:
        return calibrate(
            args.earfcn, hz, args.gain, groups, rf_args,
            args.calibrate_passes, max(args.miss_limit, 5),
        )

    say("groups %s, strongest cell in each, then repeat. Ctrl+C stops." % " ".join(str(group) for group in groups))
    say("timeouts: %.0f s search, %.0f s SIB1, bail after %d empty reports"
        % (args.search_timeout, args.sib1_timeout, args.miss_limit))
    if args.cell_id:
        say("known cells: %s" % ", ".join(str(cell) for cell in args.cell_id))
    known = set(args.cell_id)
    try:
        while True:
            for group in groups:
                read_group(
                    args.earfcn, hz, args.gain, group, known,
                    search_s=args.search_timeout,
                    sib1_s=args.sib1_timeout,
                    miss_limit=args.miss_limit,
                    rf_args=rf_args,
                )
            say("pass done")
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
