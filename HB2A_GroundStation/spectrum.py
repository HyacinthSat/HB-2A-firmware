"""Live IQ spectrum using FFTW Welch and Matplotlib's Tk canvas."""
import bootstrap  # noqa: F401 - configure native DLL paths first
import time
import tkinter as tk
from tkinter import ttk
import ctypes as ct
import os
from pathlib import Path

import numpy as np
from scipy import signal
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk


HERE = Path(__file__).resolve().parent
_native = None
try:
    path = HERE / "build" / ("hb2a_dsp.dll" if os.name == "nt" else "libhb2a_dsp.so")
    _native = ct.CDLL(str(path))
    _native.hb2a_welch_psd.argtypes = [ct.POINTER(ct.c_float), ct.c_size_t, ct.c_size_t,
                                       ct.c_float, ct.POINTER(ct.c_float), ct.POINTER(ct.c_float)]
    _native.hb2a_welch_psd.restype = ct.c_int
except (OSError, AttributeError):
    _native = None


def power_spectrum(iq, sample_rate):
    """Return a two-sided Welch PSD, using the FFTW native path when present."""
    values = np.ascontiguousarray(iq, dtype=np.complex64)
    nperseg = min(2048, len(values))
    if _native is not None and nperseg:
        raw = values.view(np.float32)
        frequency = np.empty(nperseg, dtype=np.float32)
        density = np.empty(nperseg, dtype=np.float32)
        rc = _native.hb2a_welch_psd(raw.ctypes.data_as(ct.POINTER(ct.c_float)), len(values),
                                    nperseg, float(sample_rate),
                                    frequency.ctypes.data_as(ct.POINTER(ct.c_float)),
                                    density.ctypes.data_as(ct.POINTER(ct.c_float)))
        if rc == 0:
            return frequency, density
    frequency, density = signal.welch(values, fs=sample_rate, nperseg=nperseg,
                                      return_onesided=False, detrend=False, scaling="density")
    return np.fft.fftshift(frequency), 10 * np.log10(np.maximum(np.fft.fftshift(density), 1e-20))


class SpectrumWindow:
    def __init__(self, parent, snapshot):
        self.snapshot = snapshot
        self.window = tk.Toplevel(parent)
        self.window.title("HB-2A RX Spectrum")
        self.window.geometry("980x680")
        self.window.minsize(640, 480)
        self.window.rowconfigure(1, weight=1)
        self.window.columnconfigure(0, weight=1)
        self.info = tk.StringVar(value="Waiting for IQ")
        self.paused, self.autoscale = tk.BooleanVar(), tk.BooleanVar(value=True)
        bar = ttk.Frame(self.window, padding=8)
        bar.grid(row=0, column=0, sticky="ew")
        label = ttk.Label(bar, textvariable=self.info, wraplength=600)
        label.pack(fill="x")
        bar.bind("<Configure>", lambda event: label.configure(wraplength=max(200, event.width - 16)))
        controls = ttk.Frame(bar)
        controls.pack(fill="x")
        ttk.Checkbutton(controls, text="Pause", variable=self.paused).pack(side="left")
        ttk.Checkbutton(controls, text="Auto level", variable=self.autoscale).pack(side="left")
        self.figure = Figure(figsize=(9, 5), dpi=100, constrained_layout=True)
        self.psd, self.waterfall = self.figure.subplots(2, 1, sharex=True)
        self.psd.set_ylabel("PSD / dB re 1 IQ unit^2/Hz")
        self.psd.grid(True, alpha=.25)
        self.psd.set_ylim(-150, -20)
        self.line, = self.psd.plot([], [], color="#137e98", linewidth=.8)
        self.waterfall.set_xlabel("Frequency / MHz")
        self.waterfall.set_ylabel("Recent spectra")
        self.waterfall.set_yticks([0, 119])
        self.waterfall.set_yticklabels(["Newest", "Oldest"])
        self.rows = np.full((120, 2048), np.nan)
        self.image = self.waterfall.imshow(self.rows, origin="upper", aspect="auto", cmap="viridis",
                                           vmin=-150, vmax=-20, extent=(0, 1, 119.5, -.5))
        self.canvas = FigureCanvasTkAgg(self.figure, master=self.window)
        self.canvas.get_tk_widget().grid(row=1, column=0, sticky="nsew")
        tools = ttk.Frame(self.window)
        tools.grid(row=2, column=0, sticky="ew")
        self.toolbar = NavigationToolbar2Tk(self.canvas, tools)
        self.last_total, self.session = -1, None
        self.closed, self.timer = False, None
        self.window.protocol("WM_DELETE_WINDOW", self.close)
        self.refresh()

    def show(self):
        self.window.deiconify()
        self.window.lift()

    def refresh(self):
        if self.closed:
            return
        try:
            state = self.snapshot()
            if state is None:
                self.info.set("Receiver stopped / connecting")
            else:
                session, fs, center, total, received_at, iq = state
                age = None if received_at is None else time.monotonic() - received_at
                status = "Waiting for IQ" if age is None else ("IQ active" if age < 3 else "No new IQ for %.1f s" % age)
                level = 10 * np.log10(max(float(np.mean(np.abs(iq) ** 2)), 1e-20)) if len(iq) else None
                self.info.set("%s | %.3f Msps | %s samples%s" %
                              (status, fs / 1e6, format(total, ","),
                               " | RMS %.1f dB re 1" % level if level is not None else ""))
                if session is not self.session:
                    self.session, self.last_total = session, -1
                    self.rows[:] = np.nan
                if not self.paused.get() and total != self.last_total and len(iq) >= 2048:
                    freq, density = power_spectrum(iq, fs)
                    freq = (freq + center) / 1e6
                    self.line.set_data(freq, density)
                    if self.last_total == -1:
                        self.psd.set_xlim(freq[0], freq[-1])
                    self.rows[1:] = self.rows[:-1].copy()
                    self.rows[0] = density
                    self.image.set_data(self.rows)
                    self.image.set_extent((freq[0], freq[-1], 119.5, -.5))
                    if self.autoscale.get():
                        low = float(np.percentile(density, 10)) - 10
                        high = max(low + 40, float(np.max(density)) + 10)
                        self.psd.set_ylim(low, high)
                        self.image.set_clim(low, high)
                    self.last_total = total
                    self.canvas.draw_idle()
        except Exception as exc:
            self.info.set("Spectrum error: " + str(exc))
        self.timer = self.window.after(250, self.refresh)

    def close(self):
        self.closed = True
        if self.timer is not None:
            self.window.after_cancel(self.timer)
        self.window.destroy()
