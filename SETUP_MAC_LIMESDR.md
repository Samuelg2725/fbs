# Running on a Mac (Apple Silicon) with a LimeSDR-USB

This is the srsRAN-free path. It captures from the LimeSDR through SoapySDR and
decodes PCI, MIB and SIB1 in pure Python, so there is **no srsRAN / `pdsch_ue`
build** — which is the part that is painful on macOS. The entry point is
`lte_scan.py`; the LimeSDR driver lives in `soapy_capture.py`.

Trade-off: the Python decoder is the same maths srsRAN uses, but it is less
hardened against very weak or noisy signals. For reading cell identities off
macro cells it is fine. If you later want srsRAN's robustness, run the existing
`hackrf_lte_decode.py` / `osprey-2/srsran_watch` path on a Linux box.

## 1. Install the tools (Homebrew)

```bash
# Homebrew itself, if you don't have it
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# Radio stack + Python
brew install python soapysdr limesuite
brew install pothosware/pothos/soapysdr        # only if the line above lacks the python bindings
```

`limesuite` provides the SoapySDR `lime` driver (SoapyLMS7). On Apple Silicon
everything above is native arm64.

## 2. Python packages

The decoder needs `numpy` and `scipy`, plus the SoapySDR Python bindings. The
bindings come from Homebrew's `soapysdr`, not from pip — so use a venv that can
see system site-packages, or install into the Homebrew Python.

```bash
python3 -m venv --system-site-packages ~/fbs-venv
source ~/fbs-venv/bin/activate
pip install numpy scipy
```

Check the SoapySDR binding is importable inside the venv:

```bash
python3 -c "import SoapySDR; print(SoapySDR.getAPIVersion())"
```

If that import fails, the Homebrew bindings aren't on the path. Find them with
`find $(brew --prefix) -name "SoapySDR*" -path "*site-packages*"` and add that
directory to `PYTHONPATH`, or recreate the venv with `--system-site-packages`.

## 3. Plug in the LimeSDR-USB and confirm it's seen

```bash
SoapySDRUtil --find           # should list driver=lime with your board's serial
python3 lte_scan.py --list-devices
```

If `SoapySDRUtil --find` shows nothing: unplug/replug, use a USB-3 port and a
good cable, and run `LimeUtil --update` once to flash the board's gateware.

## 4. Run it

```bash
# prove the decode chain works end to end, no radio needed
python3 lte_scan.py --self-test

# scan a known UK Band 20 downlink (Vodafone ~806 MHz)
python3 lte_scan.py --freq-mhz 806.0

# or give a channel number instead of a frequency
python3 lte_scan.py --earfcn 6300

# save the raw capture so you can re-decode it offline
python3 lte_scan.py --freq-mhz 806.0 --save vodafone.cf32
python3 lte_scan.py --file vodafone.cf32 --freq-mhz 806.0
```

Useful flags: `--gain 50` (raise for a weak cell), `--seconds 0.2` (longer
capture, more SIB1 repeats), `--antenna LNAL`/`LNAH` (force the RX port),
`--cell-id <n>` (flag a match against a known 28-bit identity),
`--device-args serial=0009...` (pick one board when several are plugged in).

## 5. Sample rate = how wide a cell you can read

SIB1 is spread across the whole cell, so the capture has to cover it. PCI is
always reported; only the identity needs the full width.

| `--rate`   | Cells it covers        | Typical UK use                 |
|------------|------------------------|--------------------------------|
| `7.68e6`   | up to 5 MHz (25 RB)    | small / rural carriers         |
| `15.36e6`  | up to 15 MHz (75 RB)   | **default**, most macro cells  |
| `30.72e6`  | 20 MHz (100 RB)        | wide carriers (needs USB 3)    |

```bash
python3 lte_scan.py --freq-mhz 806.0 --rate 30.72e6     # for a 20 MHz cell
```

If the capture is narrower than the cell, you'll get the PCI and a status line
saying SIB1 isn't in the capture — bump `--rate` and try again.

## Known UK Band 20 downlink centres (791–821 MHz)

- EE ~798.5 MHz
- Vodafone ~806.0 MHz
- O2 ~816.0 MHz

The frequency you pass must be the **centre** of the cell, not the edge of the
operator's block. `cellular_resolver.py` converts any EARFCN to its exact
downlink centre if you prefer `--earfcn`.
