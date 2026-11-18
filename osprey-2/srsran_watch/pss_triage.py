"""Fast PSS presence check for cellids.py (the optional --triage gate).

It grabs a short IQ capture through SoapySDR and matched-filters it against the
three LTE PSS sequences -- the very signal srsRAN itself syncs to. It returns
the best correlation peak-to-RMS ratio; cellids.py skips a channel only when
that ratio is below the threshold, i.e. the channel is confidently empty.

Accuracy-first by design:
  * It looks for the same PSS srsRAN needs, so it cannot reject a channel that
    srsRAN could have synced to, as long as the threshold is set below the
    weakest real cell's ratio (that is what --triage-threshold 0 calibrates).
  * Every failure path (no SoapySDR, no numpy, a capture error) RAISES, so the
    caller falls back to a full scan. The triage never silently skips.

Capture is at 1.92 MHz: the PSS occupies the centre ~945 kHz, which fits, and
the small sample count keeps the correlation fast (well under a second).
"""

from __future__ import annotations

import numpy as np

_PSS_ROOTS = (25, 29, 34)       # the three N_id_2 Zadoff-Chu roots
_CAP_RATE = 1_920_000           # 1.92 MHz: PSS fits, little data to crunch
_CAP_SECONDS = 0.25             # ~25 half-frames: several PSS repeats
_FFT = 128                      # 1.92e6 / 15 kHz subcarrier spacing


def _pss_time(root):
    """Unit-energy time-domain PSS reference of length _FFT for one ZC root."""
    d = np.empty(62, dtype=np.complex128)
    lo = np.arange(0, 31)
    hi = np.arange(31, 62)
    # 3GPP 36.211 6.11.1.1
    d[lo] = np.exp(-1j * np.pi * root * lo * (lo + 1) / 63.0)
    d[hi] = np.exp(-1j * np.pi * root * (hi + 1) * (hi + 2) / 63.0)
    freq = np.zeros(_FFT, dtype=np.complex128)
    freq[1:32] = d[31:62]            # subcarriers +1..+31
    freq[_FFT - 31:_FFT] = d[0:31]   # subcarriers -31..-1 (DC skipped)
    t = np.fft.ifft(freq)
    t /= np.sqrt(np.vdot(t, t).real)
    return t.astype(np.complex64)


_TEMPLATES = [_pss_time(r) for r in _PSS_ROOTS]


def pss_ratio(samples):
    """Best PSS matched-filter peak-to-RMS ratio across the three roots.

    Pure noise lands around 5-6 (extreme-value statistics of the filter output);
    a real cell's PSS stands well above that. 0.0 means too few samples.
    """
    x = np.asarray(samples, dtype=np.complex64)
    if x.size < _FFT * 8:
        return 0.0
    x = x - x.mean()
    rms = np.sqrt(np.mean(x.real ** 2 + x.imag ** 2)) + 1e-12
    x = (x / rms).astype(np.complex64)
    nfft = 1 << int(np.ceil(np.log2(x.size + _FFT)))
    spectrum = np.fft.fft(x, nfft)
    best = 0.0
    for tmpl in _TEMPLATES:
        matched = np.fft.fft(np.conj(tmpl[::-1]), nfft)
        corr = np.abs(np.fft.ifft(spectrum * matched))[: x.size]
        peak = float(corr.max())
        rms_corr = float(np.sqrt(np.mean(corr ** 2))) + 1e-12
        best = max(best, peak / rms_corr)
    return best


def capture(device_spec, hz, gain, rate=_CAP_RATE, seconds=_CAP_SECONDS):
    """A short IQ capture through SoapySDR. Raises on any failure.

    device_spec is the same markup cellids.py hands pdsch_ue, e.g.
    'driver=lime' or 'driver=plutosdr'. The device is fully released on return
    so pdsch_ue can open it straight afterwards.
    """
    import SoapySDR  # noqa: PLC0415
    from SoapySDR import SOAPY_SDR_CF32, SOAPY_SDR_OVERFLOW, SOAPY_SDR_RX  # noqa: PLC0415

    dev = SoapySDR.Device(device_spec)
    try:
        dev.setSampleRate(SOAPY_SDR_RX, 0, float(rate))
        dev.setFrequency(SOAPY_SDR_RX, 0, float(hz))
        try:
            dev.setGain(SOAPY_SDR_RX, 0, float(gain))
        except Exception:
            pass
        stream = dev.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32, [0])
        dev.activateStream(stream)
        try:
            total = int(rate * seconds)
            # Drop a warm-up so the front end settles before the frames we keep.
            warm = int(rate * 0.05)
            scratch = np.zeros(1 << 14, np.complex64)
            got = 0
            while got < warm:
                status = dev.readStream(stream, [scratch], len(scratch), timeoutUs=500000)
                if status.ret <= 0:
                    break
                got += status.ret
            out = np.zeros(total, np.complex64)
            filled = 0
            stalls = 0
            while filled < total:
                chunk = np.zeros(min(1 << 14, total - filled), np.complex64)
                status = dev.readStream(stream, [chunk], len(chunk), timeoutUs=500000)
                if status.ret == SOAPY_SDR_OVERFLOW:
                    continue
                if status.ret <= 0:
                    stalls += 1
                    if stalls > 10:
                        break
                    continue
                out[filled:filled + status.ret] = chunk[: status.ret]
                filled += status.ret
            if filled < _FFT * 8:
                raise RuntimeError("capture returned too few samples (%d)" % filled)
            return out[:filled]
        finally:
            dev.deactivateStream(stream)
            dev.closeStream(stream)
    finally:
        del dev


def present(device_spec, hz, gain, threshold):
    """Return (is_present, ratio).

    Raises on any capture/analysis failure so the caller can fall back to a
    full scan instead of skipping. A threshold <= 0 means 'measure only':
    is_present is always True, so nothing is skipped (calibration mode).
    """
    samples = capture(device_spec, hz, gain)
    ratio = pss_ratio(samples)
    if threshold <= 0:
        return True, ratio
    return ratio >= threshold, ratio
