"""Record one LTE capture from a LimeSDR-USB (or any SoapySDR radio).

This is the macOS / LimeSDR counterpart to hackrf_capture. It reaches the
radio through SoapySDR, so the same code opens a LimeSDR-USB, a LimeSDR Mini,
a HackRF, or anything else with a Soapy driver. The LimeSDR is the default.

The LMS7002M has its own DC-offset and I/Q imbalance correction, and LTE never
uses the centre (DC) subcarrier, so the radio is tuned straight onto the cell.
That is simpler than the HackRF low-IF trick. A low-IF offset is still offered
for a radio whose DC spike is large: it reuses hackrf_capture.condition.
"""

from __future__ import annotations

import time

import numpy as np

from hackrf_capture import condition

DEFAULT_DRIVER = "lime"
DEFAULT_RATE = 15_360_000  # covers LTE cells up to 15 MHz; use 30.72e6 for 20 MHz
DEFAULT_SECONDS = 0.120     # 12 LTE frames: enough repeats of MIB and SIB1
DEFAULT_GAIN = 45.0         # overall RX gain in dB; raise for a weak cell


def rx_antenna(driver: str, tune_hz: float) -> str | None:
    """Pick the LimeSDR RX port for a frequency. None lets the driver choose.

    The LimeSDR-USB has three RX inputs. LNAL is the low band, LNAH the high
    band, and LNAW a wideband path. The 1.5 GHz split matches the board.
    """
    if driver != "lime":
        return None
    return "LNAL" if float(tune_hz) < 1_500_000_000 else "LNAH"


def capture(
    centre_hz: float,
    sample_rate: float = DEFAULT_RATE,
    seconds: float = DEFAULT_SECONDS,
    gain: float = DEFAULT_GAIN,
    driver: str = DEFAULT_DRIVER,
    antenna: str | None = None,
    device_args: str | None = None,
    low_if_hz: float = 0.0,
):
    """One baseband capture. centre_hz is the LTE downlink centre.

    Returns the conditioned baseband samples and the Unix time taken just
    before the stream starts. low_if_hz is 0 for the LimeSDR; pass a non-zero
    value only for a radio that needs the tune moved off the cell centre.
    """
    count = int(round(float(sample_rate) * float(seconds)))
    tune = float(centre_hz) + float(low_if_hz)
    if antenna is None:
        antenna = rx_antenna(driver, tune)
    raw, capture_timestamp = _soapy(
        tune, float(sample_rate), count, float(gain), driver, antenna, device_args,
    )
    return condition(raw, sample_rate, low_if_hz), capture_timestamp


def save_capture(path, samples) -> None:
    """Write complex64 baseband samples, the format lte_scan --file reads."""
    np.asarray(samples, dtype=np.complex64).tofile(path)


def _build_args(driver: str, device_args: str | None) -> dict:
    args = {"driver": driver}
    if device_args:
        for pair in device_args.split(","):
            if "=" in pair:
                key, value = pair.split("=", 1)
                args[key.strip()] = value.strip()
    return args


def find_devices(driver: str | None = None) -> list:
    """List the Soapy radios present, for a 'is my LimeSDR seen?' check."""
    import SoapySDR  # noqa: PLC0415

    query = {} if driver is None else {"driver": driver}
    return [dict(result) for result in SoapySDR.Device.enumerate(query)]


def _soapy(tune_hz, sample_rate, count, gain, driver, antenna, device_args):
    import SoapySDR  # noqa: PLC0415
    from SoapySDR import SOAPY_SDR_CF32, SOAPY_SDR_RX

    device = SoapySDR.Device(_build_args(driver, device_args))
    channel = 0
    device.setSampleRate(SOAPY_SDR_RX, channel, float(sample_rate))
    if antenna:
        try:
            device.setAntenna(SOAPY_SDR_RX, channel, antenna)
        except Exception:
            pass
    # An analog low-pass a little wider than the sample rate keeps the whole
    # cell and rejects the neighbours.
    try:
        device.setBandwidth(SOAPY_SDR_RX, channel, min(float(sample_rate) * 1.15, 40e6))
    except Exception:
        pass
    device.setFrequency(SOAPY_SDR_RX, channel, float(tune_hz))
    try:
        device.setGain(SOAPY_SDR_RX, channel, float(gain))
    except Exception:
        pass
    # Let the LMS7 strip its own DC offset and I/Q imbalance.
    for setter, value in (("setDCOffsetMode", True), ("setIQBalanceMode", True)):
        method = getattr(device, setter, None)
        if method is not None:
            try:
                method(SOAPY_SDR_RX, channel, value)
            except Exception:
                pass

    stream = device.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32, [channel])
    output = np.zeros(count, dtype=np.complex64)
    device.activateStream(stream)
    # Drop the first ~0.15 s. The front end settles after the stream starts and
    # the LMS7 Rx calibration leaves a transient; keeping it would sit on the
    # first frames and corrupt the MIB. Discarding it also lets the USB link
    # reach steady state before the frames that matter are read.
    flush = int(0.15 * float(sample_rate))
    discarded = 0
    while discarded < flush:
        warmup = np.zeros(min(1 << 16, flush - discarded), dtype=np.complex64)
        status = device.readStream(stream, [warmup], len(warmup), timeoutUs=1_000_000)
        if status.ret <= 0:
            break
        discarded += status.ret
    capture_timestamp = time.time()
    filled = 0
    try:
        while filled < count:
            chunk = np.zeros(min(1 << 16, count - filled), dtype=np.complex64)
            status = device.readStream(stream, [chunk], len(chunk), timeoutUs=1_000_000)
            if status.ret <= 0:
                raise RuntimeError("%s stream stopped (%s)" % (driver, status.ret))
            output[filled:filled + status.ret] = chunk[: status.ret]
            filled += status.ret
    finally:
        device.deactivateStream(stream)
        device.closeStream(stream)
    return output, capture_timestamp
