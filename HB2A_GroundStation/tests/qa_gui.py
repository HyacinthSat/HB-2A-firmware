"""Geometry and next-business checks without touching SDR/audio/CAT."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import tkinter as tk
from unittest.mock import patch
from station import App, parser

root = tk.Tk()
app = App(root, parser().parse_args([]))
assert app.manual_button.cget("text") == "Manual test EXPR"
assert app.tx_mode.get() == "IC-9700 USB audio"
with patch("station.output_devices", return_value=[("0", "Speakers"), ("3", "USB Audio CODEC")]):
    app.refresh_audio()
    assert str(app.audio_combo.cget("state")) == "readonly"
    assert app.selected_audio() == "3"
    assert "USB Audio CODEC" in app.audio_choice.get()
for width, height in ((1120, 780), (940, 740)):
    root.geometry("%dx%d" % (width, height))
    root.update()
    def check(widget):
        for child in widget.winfo_children():
            if child.winfo_ismapped():
                x = child.winfo_rootx() - root.winfo_rootx()
                y = child.winfo_rooty() - root.winfo_rooty()
                assert x >= 0 and y >= 0, str(child)
                assert x + child.winfo_width() <= root.winfo_width(), str(child)
                assert y + child.winfo_height() <= root.winfo_height(), str(child)
                check(child)
    check(root)
app.next_job.set("REQF")
app.file_index.set("4")
assert app.parameters() == ("REQF", ["4"])
assert app.next_job.get() == "CAPTR"
app.camera.set("B")
assert app.parameters() == ("CAPTR", ["B", "640", "480", "5"])
root.destroy()
print("GUI geometry and one-shot next business: passed")
