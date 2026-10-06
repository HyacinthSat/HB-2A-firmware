"""Continuous SDR / USB-audio ground station."""
import bootstrap  # noqa: F401 - configure DLL paths before native dependencies

import argparse
import gc
import os
from pathlib import Path
import queue
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from PIL import Image, ImageTk

from gnuradio import blocks, gr
from dsp import Receiver
from negotiation import BusinessLoop
from radio import Rig, Transmitter
from recording import Recorder
from wire import RATES, Wire, ssdv_packet
from audio_devices import output_devices, resolve_output
from hardware_radio import HardwareTransmitter

try:
    from serial.tools import list_ports
except ImportError:
    try:
        from hardware_radio import serial as _serial
        from serial.tools import list_ports
    except ImportError:
        list_ports = None

HERE = Path(__file__).resolve().parent
CAMERA_PRESETS = ((128, 96), (256, 192), (320, 240), (480, 272), (640, 480), (800, 480))


def validate_capture_dimensions(width, height):
    """Validate the current OV5640/SSDV dimensions accepted by the flight code."""
    try:
        width, height = int(width), int(height)
    except (TypeError, ValueError) as exc:
        raise ValueError("CAPTR width and height must be integers") from exc
    if (width, height) not in CAMERA_PRESETS:
        raise ValueError("CAPTR size must be a preset: " +
                         ", ".join("%dx%d" % size for size in CAMERA_PRESETS))
    return width, height


class Archive:
    def __init__(self, output, callsign, config):
        self.directory = Path(output) / (time.strftime("%Y%m%d-%H%M%S") + "-%06d" % (time.time_ns() % 1000000))
        self.recorder = Recorder(self.directory, config)
        self.callsign, self.wire, self.images = callsign, Wire(), {}
        self.generation = 0
        self.image_states, self.updates = {}, {}

    def begin_business(self):
        self.flush(force=True)
        if self.images:
            self.generation += 1
            self.images.clear()
            self.image_states.clear()

    def close(self):
        self.flush(force=True)
        self.recorder.close()

    def save(self, group):
        saved = []
        for frame in group["frames"]:
            if frame["type"] != "image":
                continue
            image_id = frame["image"]
            image = self.images.setdefault(image_id, {})
            if frame["packet"] == 0 and image.get(0) not in (None, frame["data"]):
                saved.extend(self.flush(force=True))
                image = self.images[image_id] = {}
                self.image_states.pop(image_id, None)
                self.generation += 1
            state = self.image_states.setdefault(image_id, dict(
                generation=self.generation, eoi_packet=None, dirty=False, next_preview=0.0))
            if image.get(frame["packet"]) == frame["data"]:
                continue
            image[frame["packet"]] = frame["data"]
            state["dirty"] = True
            if frame["eoi"]:
                state["eoi_packet"] = frame["packet"]
                state["next_preview"] = 0.0
        return saved + self.flush()

    def flush(self, force=False):
        """Decode snapshots before EOI too; rebuilding admits late/out-of-order packets.

        get_jpeg fills gaps and finalizes the native decoder, so each preview
        uses a fresh decoder. Limit refresh to twice per second per image.
        """
        saved, now = [], time.monotonic()
        for image_id, state in self.image_states.items():
            if not state["dirty"] or (not force and now < state["next_preview"]):
                continue
            image = self.images[image_id]
            eoi = state["eoi_packet"]
            last_packet = max(image) if eoi is None else eoi
            missing = sorted(set(range(last_packet + 1)) - set(image))
            complete = eoi is not None and not missing
            stem = self.directory / ("image-%04d-%03d" % (state["generation"], image_id))
            path = str(stem) + (".jpg" if complete else "-partial.jpg")
            state["dirty"], state["next_preview"] = False, now + .5
            try:
                packets = [ssdv_packet(image[i], self.callsign, self.wire) for i in sorted(image)]
                stem.with_suffix(".ssdv").write_bytes(b"".join(packets))
                data = self.wire.jpeg(packets)
                temporary = Path(path + ".tmp")
                temporary.write_bytes(data)
                temporary.replace(path)
            except (ValueError, OSError) as exc:
                self.recorder.event("image_decode_error", image=image_id, error=str(exc))
                continue
            details = dict(image=image_id, generation=state["generation"], packets=len(image),
                           missing_packets=missing, eoi=eoi is not None, complete=complete)
            self.updates[path] = details
            self.recorder.event("image_saved" if complete else "image_preview",
                                path=Path(path).name, **details)
            saved.append(path)
        return saved


def make_source(args):
    if args.iq_file:
        return blocks.file_source(gr.sizeof_gr_complex, str(Path(args.iq_file).resolve()), False)
    import osmosdr
    source = None
    try:
        source = osmosdr.source(args.sdr or ("soapy=0,driver=plutosdr,uri=" + args.pluto_uri))
        actual = float(source.set_sample_rate(args.sample_rate))
        if abs(actual - args.sample_rate) > 1:
            raise ValueError("SDR sample rate is %s, expected %s" % (actual, args.sample_rate))
        args.sample_rate = actual
        source.set_center_freq(args.frequency)
        if args.sdr:
            source.set_freq_corr(args.ppm)
            source.set_gain_mode(False)
            source.set_gain(args.gain)
        else:
            source.set_center_freq(args.frequency * (1 + args.ppm / 1e6))
            source.set_bandwidth(min(400000, int(actual * .8)))
            source.set_gain_mode(True)
            # SoapyPlutoSDR owns the libiio context. Opening a second context
            # races device discovery on some Pluto firmware and causes intermittent
            # "no device context found" failures at startup.
            args.rx_device = {"uri": args.pluto_uri, "driver": "SoapyPlutoSDR",
                              "sample_rate": source.get_sample_rate(),
                              "bandwidth": source.get_bandwidth(),
                              "agc": "driver automatic"}
        return source
    except Exception:
        # A partially configured osmosdr block can retain the network libiio
        # context until Python's cyclic-GC runs. Release it before allowing a
        # retry, otherwise the next attempt reports "no device context found".
        if source is not None:
            try:
                source.unlock()
            except Exception:
                pass
        gc.collect()
        raise


def release_source(source):
    """Drop a failed source promptly so a Pluto retry gets a fresh context."""
    if source is None:
        return
    try:
        source.unlock()
    except Exception:
        pass
    gc.collect()


class App:
    def __init__(self, root, args):
        self.root, self.args = root, args
        self.events = queue.Queue(1024)
        self.receiver = self.transmitter = self.business = self.archive = None
        self.spectrum = None
        self.manual_test_pending = False
        self.counts = [0] * 6
        self.auto_requested, self.retry_at = False, 0
        root.title("HB-2A Ground Station")
        root.geometry("1120x780")
        root.minsize(940, 740)
        root.columnconfigure(0, weight=1)
        root.rowconfigure(3, weight=1)
        self.status, self.next_job = tk.StringVar(value="Stopped"), tk.StringVar(value="CAPTR")
        self.vars = {}
        config = ttk.Frame(root, padding=12)
        config.grid(row=0, column=0, sticky="ew")
        for col in (1, 3, 5):
            config.columnconfigure(col, weight=1)
        settings = [("Pluto URI", "pluto_uri", args.pluto_uri),
                    ("RX frequency / Hz", "frequency", args.frequency),
                    ("RX offset / Hz", "offset", args.offset), ("CAT host", "rig_host", args.rig_host),
                    ("CAT port", "rig_port", args.rig_port)]
        for i, (label, name, value) in enumerate(settings):
            row, col = divmod(i, 3)
            ttk.Label(config, text=label).grid(row=row, column=col * 2, sticky="w", padx=(0, 5), pady=4)
            var = self.vars[name] = tk.StringVar(value=str(value))
            ttk.Entry(config, textvariable=var, width=20).grid(row=row, column=col * 2 + 1, sticky="ew", padx=(0, 10))
        ttk.Label(config, text="USB audio output").grid(row=2, column=0, sticky="w")
        self.audio_choice = tk.StringVar()
        self.audio_devices = {}
        self.audio_combo = ttk.Combobox(config, textvariable=self.audio_choice, state="readonly")
        self.audio_combo.grid(row=2, column=1, columnspan=4, sticky="ew", padx=(0, 10))
        ttk.Button(config, text="Refresh", command=self.refresh_audio).grid(row=2, column=5, sticky="ew")
        ttk.Label(config, text="TX backend").grid(row=3, column=0, sticky="w", pady=4)
        self.tx_mode = tk.StringVar(value=args.tx_mode)
        ttk.Combobox(config, textvariable=self.tx_mode, state="readonly",
                     values=("IC-9700 USB audio", "Hardware transceiver")).grid(
                         row=3, column=1, sticky="ew", padx=(0, 10))
        ttk.Label(config, text="Hardware COM").grid(row=3, column=2, sticky="w")
        self.hardware_port = tk.StringVar(value=args.hardware_port)
        self.hardware_ports = {}
        self.hardware_combo = ttk.Combobox(config, textvariable=self.hardware_port, state="readonly")
        self.hardware_combo.grid(row=3, column=3, columnspan=2, sticky="ew", padx=(0, 10))
        ttk.Button(config, text="Refresh", command=self.refresh_hardware_ports).grid(row=3, column=5, sticky="ew")
        if os.name != "nt":
            self.audio_combo.configure(state="normal")
            self.audio_choice.set(args.audio)
        bar = ttk.Frame(root, padding=(12, 0, 12, 10))
        bar.grid(row=1, column=0, sticky="ew")
        for text, command in [("Receive", self.start_rx), ("Start automatic", self.start_auto),
                              ("Stop automatic", self.stop_auto), ("Manual test EXPR", self.manual_test),
                              ("Spectrum", self.show_spectrum),
                              ("Open IQ", self.open_iq), ("Stop all", self.stop)]:
            button = ttk.Button(bar, text=text, command=command)
            button.pack(side="left", padx=(0, 6))
            if text == "Manual test EXPR":
                self.manual_button = button
        ttk.Label(bar, textvariable=self.status).pack(side="right")
        self.table = ttk.Treeview(root, columns=("rate", "esn0", "ebn0", "frames", "seen", "probe"), show="headings", height=6)
        self.table.grid(row=2, column=0, sticky="ew", padx=12)
        for key, title in zip(self.table["columns"], ("Rate / bit/s", "Last Es/N0 / dB", "Last Eb/N0 / dB", "Packets", "Last received", "Current probe")):
            self.table.heading(key, text=title)
            self.table.column(key, width=140, anchor="center")
        for i, rate in enumerate(RATES):
            self.table.insert("", "end", iid=str(i), values=(rate, "-", "-", 0, "-", "Listening"))
        middle = ttk.Panedwindow(root, orient="horizontal")
        middle.grid(row=3, column=0, sticky="nsew", padx=12, pady=10)
        self.logbox = tk.Text(middle, height=12, width=50, state="disabled", wrap="word", font=("Consolas", 10))
        self.preview = ttk.Label(middle, text="No image", anchor="center", width=32)
        middle.add(self.logbox, weight=3)
        middle.add(self.preview, weight=2)
        self.photo = None
        self.preview_id, self.preview_complete = None, False
        self.preview_packets = set()
        lower = ttk.Frame(root, padding=12)
        lower.grid(row=4, column=0, sticky="ew")
        for c in range(8):
            lower.columnconfigure(c, weight=1)
        ttk.Label(lower, text="Next business").grid(row=0, column=0, sticky="w")
        ttk.Radiobutton(lower, text="CAPTR", variable=self.next_job, value="CAPTR").grid(row=0, column=1)
        ttk.Radiobutton(lower, text="REQF", variable=self.next_job, value="REQF").grid(row=0, column=2)
        self.camera, self.width, self.height, self.quality, self.file_index = (tk.StringVar(value=v) for v in ("A", "640", "480", "5", "0"))
        ttk.Label(lower, text="REQF index").grid(row=0, column=3)
        ttk.Entry(lower, textvariable=self.file_index, width=8).grid(row=0, column=4)
        for i, (label, var) in enumerate((("Camera", self.camera), ("Width", self.width), ("Height", self.height), ("Quality", self.quality))):
            ttk.Label(lower, text=label).grid(row=1, column=i * 2, sticky="w", pady=10)
            ttk.Entry(lower, textvariable=var, width=8).grid(row=1, column=i * 2 + 1, sticky="ew", padx=8)
        self.level, self.invert = tk.DoubleVar(value=args.amplitude), tk.BooleanVar(value=args.invert)
        ttk.Label(lower, text="USB TX level").grid(row=2, column=0, sticky="w")
        ttk.Scale(lower, from_=.01, to=1, variable=self.level).grid(row=2, column=1, columnspan=2, sticky="ew")
        ttk.Checkbutton(lower, text="Invert audio", variable=self.invert).grid(row=2, column=3, columnspan=2)
        self.tx_power = tk.StringVar(value=str(args.tx_power))
        ttk.Label(lower, text="Hardware TX power / dBm").grid(row=2, column=5, sticky="w")
        ttk.Spinbox(lower, from_=2, to=17, textvariable=self.tx_power, width=8).grid(row=2, column=6, sticky="ew")
        self.output = tk.StringVar(value=args.output)
        ttk.Label(lower, text="Recordings").grid(row=3, column=0, sticky="w", pady=8)
        ttk.Entry(lower, textvariable=self.output).grid(row=3, column=1, columnspan=7, sticky="ew")
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.refresh_audio()
        self.refresh_hardware_ports()
        root.after(50, self.poll)

    def refresh_audio(self):
        if os.name != "nt":
            return
        previous = self.audio_devices.get(self.audio_choice.get())
        try:
            devices = output_devices()
            self.audio_devices = {"%s: %s" % item: item for item in devices}
            self.audio_combo.configure(values=list(self.audio_devices))
            self.audio_choice.set("")
            selection = resolve_output(previous[1] if previous else self.args.audio, devices)
            self.audio_choice.set("%s: %s" % selection)
        except ValueError:
            pass
        except Exception as exc:
            self.log("Audio enumeration failed: " + str(exc))

    def selected_audio(self):
        if os.name != "nt":
            return self.audio_choice.get()
        selection = self.audio_devices.get(self.audio_choice.get())
        if selection is None or selection not in output_devices():
            raise ValueError("Select a connected USB audio output from the list")
        return selection[0]

    def refresh_hardware_ports(self):
        if list_ports is None:
            return
        ports = list(list_ports.comports())
        self.hardware_ports = {"%s: %s" % (p.device, p.description): p.device for p in ports}
        self.hardware_combo.configure(values=list(self.hardware_ports))
        if self.hardware_port.get() not in self.hardware_ports:
            selected = next((key for key in self.hardware_ports if "HB2A" in key.upper() or "STM" in key.upper()),
                            next(iter(self.hardware_ports), ""))
            self.hardware_port.set(selected)

    def selected_hardware_port(self):
        if self.hardware_port.get() in self.hardware_ports:
            return self.hardware_ports[self.hardware_port.get()]
        if self.hardware_port.get() and self.hardware_port.get() in self.hardware_ports.values():
            return self.hardware_port.get()
        raise ValueError("Select the connected hardware transceiver COM port")

    def log(self, text):
        if self.archive:
            self.archive.recorder.event("status", message=text)
        self.logbox.configure(state="normal")
        self.logbox.insert("end", time.strftime("%H:%M:%S ") + text + "\n")
        if int(self.logbox.index("end-1c").split(".")[0]) > 1500:
            self.logbox.delete("1.0", "501.0")
        self.logbox.see("end")
        self.logbox.configure(state="disabled")

    def post(self, event):
        self.events.put(event, timeout=3)

    def clear_preview(self, text="Waiting for new image"):
        self.preview.configure(image="", text=text)
        self.photo = None
        self.preview_id, self.preview_complete = None, False
        self.preview_packets = set()
        self.preview_first_data = None

    def image_progress(self, group):
        for frame in group["frames"]:
            if frame["type"] != "image":
                continue
            if frame["image"] != self.preview_id or (frame["packet"] == 0 and
                    self.preview_first_data is not None and frame.get("data") != self.preview_first_data):
                self.clear_preview()
                self.preview_id = frame["image"]
            if frame["packet"] == 0:
                self.preview_first_data = frame.get("data")
            self.preview_packets.add(frame["packet"])
            if not self.preview_complete and self.photo is None:
                self.preview.configure(text="Receiving image %d\n%d packets received" %
                                       (self.preview_id, len(self.preview_packets)))

    def show_image(self, path, details=None):
        with Image.open(path) as image:
            image.thumbnail((max(100, self.preview.winfo_width() - 12),
                             max(100, self.preview.winfo_height() - 64)))
            photo = ImageTk.PhotoImage(image)
        text = ""
        if details:
            text = "Image %d: %d packets; %d missing; %s" % (
                details["image"], details["packets"], len(details["missing_packets"]),
                "complete" if details["complete"] else
                "EOI received, incomplete" if details["eoi"] else "receiving, no EOI yet")
        self.preview.configure(image=photo, text=text, compound="top", wraplength=300)
        self.photo = photo
        self.preview_complete = details["complete"] if details else True

    def show_updates(self, paths):
        for path in paths:
            details = self.archive.updates[path]
            if details["image"] != self.preview_id:
                continue
            try:
                self.show_image(path, details)
            except (ValueError, OSError) as exc:
                self.log("Image preview: " + str(exc))
            if details["eoi"]:
                self.log(("JPEG: " if details["complete"] else "Partial JPEG: ") + path)

    def refresh_probe_status(self):
        statuses = self.business.session.rate_status if self.business and self.business.enabled else {}
        for rate in range(len(RATES)):
            self.table.set(str(rate), "probe", statuses.get(rate, "Listening"))

    def spectrum_snapshot(self):
        if self.receiver is None:
            return None
        history = self.receiver.history
        return (history, self.args.sample_rate, self.args.frequency + self.args.offset,
                *history.latest())

    def show_spectrum(self):
        if self.spectrum is None or self.spectrum.closed:
            from spectrum import SpectrumWindow
            self.spectrum = SpectrumWindow(self.root, self.spectrum_snapshot)
        else:
            self.spectrum.show()

    def start_rx(self, replay=False):
        if self.receiver:
            self.show_spectrum()
            return True
        source = None
        try:
            try:
                self.show_spectrum()
            except Exception as exc:
                self.log("Spectrum unavailable: " + str(exc))
            for name, var in self.vars.items():
                value = var.get()
                setattr(self.args, name, value if name in ("pluto_uri", "rig_host") else float(value))
            self.args.rig_port = int(self.args.rig_port)
            self.args.output = self.output.get()
            self.args.tx_mode = self.tx_mode.get()
            if not replay:
                self.args.iq_file = None
            self.status.set("Connecting")
            self.log("Opening " + (self.args.iq_file or self.args.sdr or self.args.pluto_uri))
            self.root.update_idletasks()
            source = make_source(self.args)
            self.archive = Archive(self.args.output, self.args.callsign, vars(self.args).copy())
            self.clear_preview("Waiting for image")
            self.receiver = Receiver(source, self.args.sample_rate, self.post,
                                     self.args.offset, self.args.callsign,
                                     self.archive.recorder,
                                     both_polarities=self.args.both_polarities,
                                     all_codes=self.args.all_codes)
            self.receiver.start()
            self.status.set("Replaying" if replay else "Receiving")
            self.log("Recordings: " + str(self.archive.directory))
            return True
        except Exception as exc:
            if self.receiver is not None:
                try:
                    self.receiver.close()
                except Exception:
                    pass
            else:
                release_source(source)
            if self.archive:
                self.archive.recorder.event("receiver_error", error=str(exc))
                self.archive.close()
            self.receiver = self.archive = None
            self.status.set("Receiver failed")
            self.log("Receiver failed: " + str(exc))
            messagebox.showerror("Receiver", str(exc))
            return False

    def parameters(self):
        command = self.next_job.get()
        fields = [self.file_index.get()] if command == "REQF" else [v.get() for v in (self.camera, self.width, self.height, self.quality)]
        Wire().uplink(command, fields)
        if command == "CAPTR":
            validate_capture_dimensions(fields[1], fields[2])
        if self.archive:
            self.archive.begin_business()
            self.archive.recorder.business_id = self.business.number + 1
            self.archive.recorder.event("business_start", command=command, fields=fields)
        self.clear_preview()
        if command == "REQF":
            self.next_job.set("CAPTR")
        return command, fields

    def send(self, command, fields):
        if hasattr(self.transmitter, "amplitude"):
            self.transmitter.amplitude, self.transmitter.invert = self.level.get(), self.invert.get()
        if self.receiver and command in ("HSPAR", "CHPRAM"):
            try:
                # HSPAR/CHPRAM fields are rate, CodeID, IntlvID.
                self.receiver.set_profile(int(fields[1]), int(fields[2]))
                self.log("Decoder profile: code=%s depth=%s" % (fields[1], fields[2]))
            except (IndexError, ValueError, TypeError):
                self.log("Invalid outgoing decoder profile")
        self.transmitter.send(command, fields)
        self.log("TX queued: " + " ".join([command, *fields]))

    def manual_test(self):
        """Transmit one EXPR lease/test frame, without starting BusinessLoop."""
        if self.business and self.business.enabled:
            self.log("Stop automatic before using manual test")
            return
        if self.manual_test_pending:
            self.log("Manual test is already queued")
            return
        try:
            if not self.start_rx():
                return
            if self.args.iq_file:
                raise ValueError("Manual transmission requires live Pluto/SDR")
            if self.tx_mode.get() == "Hardware transceiver":
                self.args.hardware_port = self.selected_hardware_port()
                self.args.tx_power = int(self.tx_power.get())
            else:
                self.args.audio = self.selected_audio()
            if self.transmitter:
                self.transmitter.close()
            if self.tx_mode.get() == "Hardware transceiver":
                self.transmitter = HardwareTransmitter(self.post, self.args.hardware_port,
                                                       self.args.tx_power, self.archive.recorder)
            else:
                rig = Rig(self.args.rig_host, self.args.rig_port)
                rig.prepare()
                self.transmitter = Transmitter(self.post, self.args.audio, self.level.get(), self.invert.get(),
                                               rig, self.archive.recorder)
            self.transmitter.armed = True
            self.manual_test_pending = True
            self.manual_button.configure(state="disabled")
            self.transmitter.send("EXPR", [])
            self.status.set("Manual test transmitting")
            self.log("Manual test EXPR queued; no automatic business will start")
        except Exception as exc:
            self.manual_test_pending = False
            self.manual_button.configure(state="normal")
            self.log("Manual test failed: " + str(exc))
            if self.transmitter:
                self.transmitter.close()
                self.transmitter = None

    def start_auto(self):
        self.auto_requested = True
        try:
            if self.business and self.business.enabled:
                return
            if not self.start_rx():
                return
            if self.args.iq_file:
                raise ValueError("Automatic transmission requires live SDR")
            if self.tx_mode.get() == "Hardware transceiver":
                self.args.hardware_port = self.selected_hardware_port()
                self.args.tx_power = int(self.tx_power.get())
            else:
                self.args.audio = self.selected_audio()
            if self.transmitter:
                self.transmitter.close()
                self.transmitter = None
            if self.tx_mode.get() == "Hardware transceiver":
                self.transmitter = HardwareTransmitter(self.post, self.args.hardware_port,
                                                       self.args.tx_power, self.archive.recorder)
            else:
                rig = Rig(self.args.rig_host, self.args.rig_port)
                rig.prepare()
                self.transmitter = Transmitter(self.post, self.args.audio, self.level.get(), self.invert.get(),
                                               rig, self.archive.recorder)
            self.transmitter.armed = True
            self.business = BusinessLoop(self.send, self.log, self.parameters, self.args.esn0_threshold)
            self.business.start(time.monotonic())
        except Exception as exc:
            self.stop_auto()
            self.auto_requested, self.retry_at = True, time.monotonic() + 15
            self.log("Automatic start failed; retry in 15s: " + str(exc))

    def stop_auto(self):
        self.auto_requested = False
        if self.business:
            self.business.stop()
        if self.transmitter:
            self.transmitter.disarm()
        self.status.set("Automatic stopped")

    def open_iq(self):
        self.stop()
        path = filedialog.askopenfilename(filetypes=[("IQ", "*.sigmf-data *.cf32 *.cfile *.iq"), ("All", "*")])
        if path:
            self.args.iq_file = path
            self.start_rx(True)

    def poll(self):
        try:
            for _ in range(100):
                try:
                    group = self.events.get_nowait()
                except queue.Empty:
                    break
                if "error" in group:
                    manual_error = self.manual_test_pending
                    if manual_error:
                        self.manual_test_pending = False
                        self.manual_button.configure(state="normal")
                    resume = self.auto_requested
                    self.stop_auto()
                    self.auto_requested, self.retry_at = resume, time.monotonic() + 15
                    if manual_error:
                        self.status.set("Manual test failed")
                    self.log("ERROR: " + group["error"])
                    continue
                if "tx" in group:
                    self.log("TX complete: " + group["tx"])
                    if group["tx"] == "EXPR" and self.manual_test_pending:
                        self.manual_test_pending = False
                        self.manual_button.configure(state="normal")
                        self.status.set("Manual test sent; receiving")
                    if self.business:
                        self.business.transmitted(group["tx"], time.monotonic())
                    continue
                rate = group["rate"]
                self.counts[rate] += len(group["frames"])
                metric = lambda value: "-" if value is None else "%.2f" % value
                self.table.item(str(rate), values=(RATES[rate], metric(group["esn0"]), metric(group["ebn0"]),
                                                   self.counts[rate], time.strftime("%H:%M:%S"),
                                                   self.table.set(str(rate), "probe")))
                if self.business:
                    self.business.group(group, time.monotonic())
                for frame in group["frames"]:
                    if frame["type"] == "text":
                        self.log("RX %d: %s" % (RATES[rate], frame["text"]))
                        if self.receiver and rate == 0 and frame["text"].startswith("HSCON,"):
                            try:
                                _, selected_rate, selected_code, selected_depth = frame["text"].split(",")
                                self.receiver.set_profile(int(selected_code), int(selected_depth))
                                self.log("Decoder profile: code=%s depth=%s" %
                                         (selected_code, selected_depth))
                            except (ValueError, TypeError):
                                self.log("Invalid HSCON decoder profile")
                try:
                    self.image_progress(group)
                    self.show_updates(self.archive.save(group))
                except (ValueError, OSError) as exc:
                    self.log("Image: " + str(exc))
            if self.archive:
                self.show_updates(self.archive.flush())
            if self.business and self.business.enabled:
                self.business.tick(time.monotonic())
                self.status.set("Fix business settings; auto resumes" if self.business.state == "parameters" else
                                "Business %d: %s / %s" % (self.business.number, self.business.state, self.business.session.state))
            elif self.auto_requested and time.monotonic() >= self.retry_at:
                self.start_auto()
            self.refresh_probe_status()
        except Exception as exc:
            self.stop_auto()
            self.log("ERROR: " + str(exc))
        self.root.after(50, self.poll)

    def stop(self):
        self.stop_auto()
        self.manual_test_pending = False
        if hasattr(self, "manual_button"):
            self.manual_button.configure(state="normal")
        if self.transmitter:
            self.transmitter.close()
            self.transmitter = None
        if self.receiver:
            self.receiver.close()
            self.receiver = None
        while not self.events.empty():
            self.events.get_nowait()
        if self.archive:
            self.archive.close()
            self.archive = None
        self.status.set("Stopped")

    def close(self):
        self.stop()
        if self.spectrum and not self.spectrum.closed:
            self.spectrum.close()
        self.root.destroy()


def parser():
    p = argparse.ArgumentParser(description="HB-2A continuous SDR / IC-9700 ground station")
    p.add_argument("--sdr", help="Override Pluto with an osmosdr device string")
    p.add_argument("--pluto-uri", default="ip:192.168.1.10")
    p.add_argument("--sample-rate", type=float, default=521000)
    p.add_argument("--frequency", type=float, default=144925000)
    p.add_argument("--offset", type=float, default=0)
    p.add_argument("--gain", type=float, default=30)
    p.add_argument("--ppm", type=float, default=0)
    p.add_argument("--iq-file")
    p.add_argument("--callsign", default="BG7ZDQ")
    p.add_argument("--audio", default="USB Audio CODEC")
    p.add_argument("--amplitude", type=float, default=.25)
    p.add_argument("--invert", action="store_true")
    p.add_argument("--rig-host", default="127.0.0.1")
    p.add_argument("--rig-port", type=int, default=4532)
    p.add_argument("--output", default=str(HERE / "output"))
    p.add_argument("--esn0-threshold", type=float, default=3.5)
    p.add_argument("--headless", action="store_true", help="Receive and record only")
    p.add_argument("--tx-mode", choices=("IC-9700 USB audio", "Hardware transceiver"), default="IC-9700 USB audio")
    p.add_argument("--hardware-port", default="")
    p.add_argument("--tx-power", type=int, default=10)
    p.add_argument("--both-polarities", action="store_true",
                   help="Decode both soft-decision polarities (higher CPU load)")
    p.add_argument("--all-codes", action="store_true",
                   help="Decode CodeID 0..3 (higher CPU load; default is CodeID 1)")
    return p


def main():
    args = parser().parse_args()
    if not 3 <= args.esn0_threshold <= 3.5:
        raise SystemExit("Es/N0 threshold must be 3.0..3.5 dB")
    if args.headless:
        source = make_source(args)
        archive = Archive(args.output, args.callsign, vars(args).copy())
        events = queue.Queue()
        receiver = Receiver(source, args.sample_rate, events.put, args.offset, args.callsign,
                            archive.recorder, both_polarities=args.both_polarities,
                            all_codes=args.all_codes)
        receiver.start()
        try:
            if args.iq_file:
                receiver.wait()
            while not args.iq_file or not events.empty():
                try:
                    group = events.get(timeout=.25)
                except queue.Empty:
                    archive.flush()
                    continue
                archive.save(group)
                print("RX", RATES[group["rate"]], "Es/N0", group["esn0"], "Eb/N0", group["ebn0"],
                      [f.get("text", "SSDV") for f in group["frames"]], flush=True)
        except KeyboardInterrupt:
            pass
        finally:
            receiver.close()
            archive.close()
    else:
        root = tk.Tk()
        App(root, args)
        root.mainloop()


if __name__ == "__main__":
    main()
