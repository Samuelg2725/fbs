"""Find LTE cells and read their 28-bit identity on a Mac with a LimeSDR.

This is the srsRAN-free path. It captures from the LimeSDR through SoapySDR,
runs the pure-Python PSS/SSS search for the physical cell IDs, then decodes
MIB and SIB1 in Python to read the real cell identity. Nothing here needs
srsRAN or a built pdsch_ue, so it runs on macOS as it stands.

    python lte_scan.py --self-test
    python lte_scan.py --list-devices
    python lte_scan.py --earfcn 6300
    python lte_scan.py --freq-mhz 806.0 --save vodafone.cf32
    python lte_scan.py --file vodafone.cf32 --freq-mhz 806.0

Sample rate sets how wide a cell SIB1 can be read from:
    7.68e6  -> up to 5 MHz cells
    15.36e6 -> up to 15 MHz cells   (default)
    30.72e6 -> 20 MHz cells
A capture narrower than the cell still reports the PCI; it just cannot reach
SIB1, and the status line says so.
"""

from __future__ import annotations

import argparse
import datetime
import sys
import time

import numpy as np

from cellular_resolver import lte_carrier
from lte_cellid import decode_cell_identity
from lte_pci_search import search_samples
from soapy_capture import (
    DEFAULT_DRIVER,
    DEFAULT_GAIN,
    DEFAULT_RATE,
    DEFAULT_SECONDS,
    capture,
    find_devices,
    save_capture,
)

_RESET = "\033[0m"
_CYAN = "\033[36m"
_BOLD_GREEN = "\033[1;32m"
_BOLD_RED = "\033[1;31m"


def _report(freq_mhz, earfcn, hit, result, capture_timestamp=None, target_cell_id=None):
    if earfcn is None:
        lines = ["Frequency: %.3f MHz" % freq_mhz]
    else:
        lines = ["Frequency: %.3f MHz  EARFCN %d" % (freq_mhz, earfcn)]
    lines.append(
        "PCI %d  SSS %.2f  offset %+.1f kHz"
        % (hit["pci"], hit["sss_score"], hit["offset_hz"] / 1000.0)
    )
    if result.get("bandwidth_rb"):
        lines.append(
            "MIB: %d RB (%s MHz), PHICH %s Ng=%s, SFN %s, antennas %s"
            % (
                result["bandwidth_rb"],
                result["bandwidth_mhz"],
                result["phich_duration"],
                result["phich_resource"],
                result["sfn"],
                result["antennas"],
            )
        )
    if result.get("cell_id") is not None:
        if result.get("mcc") is not None:
            name = result.get("operator") or "unknown network"
            lines.append("Network: %s-%s %s" % (result["mcc"], result.get("mnc_digits", result.get("mnc")), name))
            lines.append("TAC: %s" % result.get("tac"))
        cell_line = "Cell ID: %s%d%s (0x%08X)" % (_CYAN, result["cell_id"], _RESET, result["cell_id"])
        if target_cell_id is not None and result["cell_id"] == target_cell_id:
            cell_line += " %s[TARGET MATCH]%s\a" % (_BOLD_GREEN, _RESET)
        elif target_cell_id is not None:
            cell_line += " %s[NO MATCH]%s" % (_BOLD_RED, _RESET)
        lines.append(cell_line)
        lines.append("eNB ID: %s%d%s" % (_CYAN, result["enb_id"], _RESET))
        lines.append("Sector: %d" % result["sector_id"])
    else:
        lines.append("Cell ID: " + result.get("cell_id_status", "not decoded"))
        if target_cell_id is not None:
            lines.append("Target cell ID %d was not found" % target_cell_id)
    if capture_timestamp is not None:
        local = datetime.datetime.fromtimestamp(capture_timestamp)
        clock = "%s.%03d" % (
            time.strftime("%H:%M:%S", time.localtime(capture_timestamp)),
            local.microsecond // 1000,
        )
        lines.append("Capture timestamp: %r (%s)" % (capture_timestamp, clock))
    return "\n".join(lines)


def decode_capture(samples, sample_rate, freq_mhz, earfcn=None, pci=None, capture_timestamp=None, target_cell_id=None):
    report = search_samples(np.asarray(samples), sample_rate)
    hits = report["hits"]
    if not hits and pci is None:
        return (
            "No LTE cell at %.3f MHz\nBest PSS %.1fx at %+.1f kHz, SSS %.2f (needs %.2f)"
            % (
                freq_mhz,
                report["best_ratio"],
                report["best_offset_hz"] / 1000.0,
                report["best_sss"],
                report["sss_min"],
            )
        )
    if hits:
        hit = hits[0]
    else:
        hit = {"pci": int(pci), "sss_score": report["best_sss"], "offset_hz": 0.0}
    print("Reading MIB and SIB1 for PCI %d..." % hit["pci"], flush=True)
    result = decode_cell_identity(
        np.asarray(samples),
        int(hit["pci"]),
        cfo_hz=float(hit["offset_hz"]),
        sample_rate=sample_rate,
    )
    text = _report(freq_mhz, earfcn, hit, result, capture_timestamp, target_cell_id)
    extras = [int(item["pci"]) for item in hits[1:]]
    if extras:
        text += "\nAlso heard PCI %s" % ", ".join(str(item) for item in extras)
    return text


def self_test() -> bool:
    """A synthetic 10 MHz cell must read back as PCI 430 with the written identity."""
    from lte_cellid import synthesize_cell

    narrow = synthesize_cell(430, 7159822, n_rb=6, ports=1)
    report = search_samples(narrow, 1_920_000)
    if not report["hits"] or report["hits"][0]["pci"] != 430:
        return False
    result = decode_cell_identity(narrow, 430, sample_rate=1_920_000)
    return bool(result.get("cell_id") == 7159822 and result.get("enb_id") == (7159822 >> 8))


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Find LTE cells and read the cell identity with a LimeSDR.")
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--list-devices", action="store_true", help="print the Soapy radios that are visible and exit")
    parser.add_argument("--earfcn", type=int)
    parser.add_argument("--freq-mhz", type=float)
    parser.add_argument("--file", help="complex64 capture recorded at --rate")
    parser.add_argument("--save", help="write the baseband complex64 capture here")
    parser.add_argument("--rate", type=float, default=DEFAULT_RATE, help="sample rate in Hz (default 15.36e6)")
    parser.add_argument("--seconds", type=float, default=DEFAULT_SECONDS)
    parser.add_argument("--gain", type=float, default=DEFAULT_GAIN, help="overall RX gain in dB")
    parser.add_argument("--driver", default=DEFAULT_DRIVER, help="Soapy driver (default lime)")
    parser.add_argument("--antenna", help="force an RX port, e.g. LNAL or LNAH")
    parser.add_argument("--device-args", help="extra Soapy device args, comma separated, e.g. 'serial=00090726...'")
    parser.add_argument("--low-if-mhz", type=float, default=0.0, help="tune this far above the cell, for a radio with a strong DC spike")
    parser.add_argument("--pci", type=int, help="use this PCI when the sync score is short")
    parser.add_argument("--cell-id", type=int, help="known 28-bit cell identity to match against this capture")
    args = parser.parse_args(argv)

    if args.self_test:
        ok = self_test()
        print("self-test %s" % ("OK" if ok else "FAILED"))
        return 0 if ok else 1

    if args.list_devices:
        try:
            devices = find_devices()
        except ImportError:
            print("SoapySDR is not installed. See SETUP_MAC_LIMESDR.md.", file=sys.stderr)
            return 1
        if not devices:
            print("No Soapy radios found. Is the LimeSDR plugged in and seen by SoapySDRUtil --find?")
            return 1
        for device in devices:
            print(", ".join("%s=%s" % (key, value) for key, value in device.items()))
        return 0

    earfcn = args.earfcn
    if earfcn is not None:
        carrier = lte_carrier(earfcn)
        if carrier is None:
            print("EARFCN %d is not a downlink channel" % earfcn, file=sys.stderr)
            return 1
        freq_mhz = carrier.downlink_mhz
    elif args.freq_mhz is not None:
        freq_mhz = float(args.freq_mhz)
    else:
        print("Pass --earfcn, --freq-mhz, --file (with --freq-mhz), --list-devices or --self-test.", file=sys.stderr)
        return 1

    capture_timestamp = None
    if args.file:
        samples = np.fromfile(args.file, dtype=np.complex64)
        print("Searching %s" % args.file, flush=True)
    else:
        try:
            samples, capture_timestamp = capture(
                freq_mhz * 1e6,
                sample_rate=args.rate,
                seconds=args.seconds,
                gain=args.gain,
                driver=args.driver,
                antenna=args.antenna,
                device_args=args.device_args,
                low_if_hz=args.low_if_mhz * 1e6,
            )
        except ImportError:
            print("SoapySDR is not installed. See SETUP_MAC_LIMESDR.md.", file=sys.stderr)
            return 1
        if args.save:
            save_capture(args.save, samples)
            print("Saved %d samples to %s" % (len(samples), args.save))

    print(decode_capture(samples, args.rate, freq_mhz, earfcn, args.pci, capture_timestamp, args.cell_id))
    return 0


if __name__ == "__main__":
    sys.exit(main())
