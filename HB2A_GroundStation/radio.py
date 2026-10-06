"""GNU Radio audio transport and Hamlib rigctld PTT (no private CI-V stack)."""
import queue
import os
import socket
import threading
import time

import bootstrap  # noqa: F401
import numpy as np
from gnuradio import audio, blocks, gr

from dsp import tx_baseband
from wire import Wire
from audio_devices import output_devices, resolve_output, validate_output


class Rig:
    def __init__(self, host="127.0.0.1", port=4532):
        self.host, self.port = host, port

    def command(self, command):
        with socket.create_connection((self.host, self.port), timeout=2) as sock:
            sock.settimeout(2)
            sock.sendall((command + "\n").encode("ascii"))
            with sock.makefile("rb") as stream:
                reply = stream.readline(1024).decode("ascii").strip()
            if not reply or (reply.startswith("RPRT") and reply != "RPRT 0"):
                raise RuntimeError("Hamlib %s: %s" % (command, reply))
            return reply

    def ptt(self, on):
        if self.command("T %d" % bool(on)) != "RPRT 0":
            raise RuntimeError("No PTT acknowledgement")

    def check_frequency(self):
        if abs(float(self.command("f")) - 435400000) > 10:
            raise RuntimeError("IC-9700 transmit VFO must be set to 435.400 MHz")

    def prepare(self):
        self.ptt(False)
        for command in ("S 0 VFOB", "F 435400000", "M PKTFM 15000"):
            if self.command(command) != "RPRT 0":
                raise RuntimeError("CAT configuration failed: " + command)
        self.check_frequency()


class Transmitter:
    def __init__(self, report, device="", amplitude=.25, invert=False, rig=None, recorder=None):
        self.report, self.device, self.amplitude, self.invert = report, device, amplitude, invert
        self.device_name = None
        if os.name == "nt":
            self.device, self.device_name = resolve_output(device, output_devices())
            gr.prefs().set_string("audio", "audio_module", "windows")
        self.rig = rig or Rig()
        self.recorder = recorder
        self.queue, self.armed, self.stop_event = queue.Queue(8), False, threading.Event()
        self.wire, self.graph = Wire(), None
        self.worker = threading.Thread(target=self.run, name="IC9700 TX", daemon=True)
        self.worker.start()

    def send(self, command, fields=()):
        if not self.armed:
            raise RuntimeError("TX is not armed")
        bits = self.wire.uplink(command, fields)
        self.queue.put_nowait((command, list(fields), bits))

    def disarm(self):
        self.armed = False
        while True:
            try:
                self.queue.get_nowait()
            except queue.Empty:
                break

    def run(self):
        while not self.stop_event.is_set():
            try:
                command, fields, bits = self.queue.get(timeout=.1)
            except queue.Empty:
                continue
            if not self.armed:
                continue
            try:
                values = tx_baseband(bits, self.amplitude, invert=self.invert)
                playback = np.r_[np.zeros(4800, np.float32), values, np.zeros(9600, np.float32)]
                if self.recorder:
                    self.recorder.save_tx(command, fields, bits, playback, amplitude=self.amplitude,
                                          invert=self.invert, audio_device=self.device, audio_name=self.device_name)
                self.rig.check_frequency()
                if self.device_name is not None:
                    validate_output(self.device, self.device_name)
                # Open the selected output before keying the radio.
                graph = gr.top_block("IC9700 DATA output")
                source = blocks.vector_source_f(playback.tolist(), False)
                sink = audio.sink(48000, self.device, True)
                graph.connect(source, sink)
                self.graph = graph
                if not self.armed or self.stop_event.is_set():
                    continue
                try:
                    self.rig.ptt(True)
                    if self.recorder:
                        self.recorder.event("ptt", on=True, command=command)
                    time.sleep(.15)
                    graph.run()
                    time.sleep(.2)
                finally:
                    graph.stop()
                    graph.wait()
                    self.rig.ptt(False)
                    if self.recorder:
                        self.recorder.event("ptt", on=False, command=command)
                self.report({"tx": command})
            except Exception as exc:
                self.disarm()
                # Retry release on a fresh TCP connection if the first failed.
                try:
                    self.rig.ptt(False)
                except Exception as release:
                    self.report({"error": "PTT release failed: %s" % release})
                self.report({"error": str(exc)})
            finally:
                self.graph = None

    def close(self):
        self.disarm()
        self.stop_event.set()
        if self.graph is not None:
            self.graph.stop()
        self.worker.join(timeout=8)
        if self.worker.is_alive():
            raise RuntimeError("TX worker did not stop; check radio PTT")

