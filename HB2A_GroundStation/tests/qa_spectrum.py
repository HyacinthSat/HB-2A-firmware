"""Render spectrum from recorded IQ without opening SDR, CAT or audio."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import bootstrap
import time
import tkinter as tk
import numpy as np
from PIL import ImageGrab
from dsp import IqHistory
from spectrum import SpectrumWindow

root = tk.Tk()
root.withdraw()
history = IqHistory(1024000)
if len(sys.argv) > 1:
    iq = np.fromfile(sys.argv[1], dtype="<c8", count=8192)
else:
    iq = np.exp(2j * np.pi * 64000 * np.arange(8192) / 1024000).astype(np.complex64)

def snapshot():
    return (history, 1024000, 144925000, *history.latest())

window = SpectrumWindow(root, snapshot)
try:
    for _ in range(5):
        history.work([iq], [np.empty_like(iq)])
        root.after(300, root.quit)
        root.mainloop()
    assert len(window.line.get_xdata()) == 2048
    assert np.isfinite(window.rows[:5]).any()
    assert "IQ active" in window.info.get()
    window.paused.set(True)
    total = window.last_total
    history.work([iq], [np.empty_like(iq)])
    root.after(300, root.quit)
    root.mainloop()
    assert window.last_total == total
    window.paused.set(False)
    output = Path(__file__).resolve().parents[1] / "build"
    output.mkdir(exist_ok=True)
    for width, height in ((980, 680), (640, 480)):
        window.window.geometry("%dx%d+20+20" % (width, height))
        root.after(500, root.quit)
        root.mainloop()
        x, y = window.window.winfo_rootx(), window.window.winfo_rooty()
        ImageGrab.grab(bbox=(x, y, x + width, y + height)).save(output / ("spectrum-%d.png" % width))
    history.last_received_at = time.monotonic() - 5
    root.after(300, root.quit)
    root.mainloop()
    assert "No new IQ" in window.info.get()
finally:
    window.close()
    root.destroy()
print("Spectrum, waterfall, pause, stale IQ and both window sizes: passed")
