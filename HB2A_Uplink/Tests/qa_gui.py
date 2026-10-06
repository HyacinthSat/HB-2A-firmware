"""Exercise the actual Tk layout at two sizes; no serial connections or RF."""
from pathlib import Path
import sys
import tkinter as tk
import time
from PIL import ImageGrab

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "Host"))
from uplink_gui import App

root = tk.Tk()
app = App(root)
output = ROOT / "Tests/build"
output.mkdir(exist_ok=True)


def check_bounds(widget):
    for child in widget.winfo_children():
        if not child.winfo_ismapped():
            continue
        x, y = child.winfo_x(), child.winfo_y()
        assert x >= 0 and y >= 0, str(child)
        assert x + child.winfo_width() <= widget.winfo_width() + 2, str(child)
        assert y + child.winfo_height() <= widget.winfo_height() + 2, str(child)
        check_bounds(child)


try:
    root.attributes("-topmost", True)
    for size in ("900x710", "740x660"):
        root.geometry(size + "+40+40")
        root.update()
        for kind in ("EXPR", "CAPTR", "REQF", "HSPAR", "CHPRAM", "CFRM", "CANCEL", "TRANPD"):
            app.kind.set(kind)
            app.change_command()
            root.update()
            check_bounds(root)
        assert str(app.send_button["state"]) == "disabled"
        app.events.put(("connected", ("TEST", "OK HB2A-UPLINK/1 RADIO=READY")))
        app.poll()
        assert str(app.send_button["state"]) == "normal"
        app.events.put(("closed", ""))
        app.poll()
        app.log.configure(state="normal")
        app.log.delete("1.0", "end")
        app.log.configure(state="disabled")
        root.update()
        time.sleep(0.2)
        x, y = root.winfo_rootx(), root.winfo_rooty()
        try:
            ImageGrab.grab(bbox=(x, y, x + root.winfo_width(), y + root.winfo_height())).save(output / f"gui-{size}.png")
        except OSError as exc:
            print(f"Desktop screenshot unavailable ({size}): {exc}")
    print("GUI bounds and connection states passed at 900x710 and 740x660")
finally:
    app.close()
