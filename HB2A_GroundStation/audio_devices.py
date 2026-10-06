"""Enumerate the same Windows waveOut devices used by GNU Radio audio."""
import ctypes
import os
from ctypes import wintypes


class WaveOutCaps(ctypes.Structure):
    _fields_ = [("manufacturer", wintypes.WORD), ("product", wintypes.WORD),
                ("version", wintypes.DWORD), ("name", wintypes.WCHAR * 32),
                ("formats", wintypes.DWORD), ("channels", wintypes.WORD),
                ("reserved", wintypes.WORD), ("support", wintypes.DWORD)]


def output_devices():
    if os.name != "nt":
        return []
    api = ctypes.WinDLL("winmm")
    api.waveOutGetNumDevs.restype = wintypes.UINT
    api.waveOutGetDevCapsW.argtypes = [ctypes.c_size_t, ctypes.POINTER(WaveOutCaps), wintypes.UINT]
    api.waveOutGetDevCapsW.restype = wintypes.UINT
    devices = []
    for index in range(api.waveOutGetNumDevs()):
        caps = WaveOutCaps()
        error = api.waveOutGetDevCapsW(index, ctypes.byref(caps), ctypes.sizeof(caps))
        if error:
            raise RuntimeError("Cannot enumerate audio output %d (waveOut error %d)" % (index, error))
        if caps.channels:
            devices.append((str(index), caps.name))
    return devices


def resolve_output(value, devices):
    exact = [(index, name) for index, name in devices if value in (index, name)]
    matches = exact or [(index, name) for index, name in devices if value and value.lower() in name.lower()]
    if len(matches) != 1:
        raise ValueError("Select a connected USB audio output; missing or ambiguous device: " + value)
    return matches[0]


def validate_output(index, expected_name):
    if (index, expected_name) not in output_devices():
        raise RuntimeError("Selected audio output changed or disconnected; refresh and select it again")
