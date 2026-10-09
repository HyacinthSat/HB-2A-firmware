"""Serial adapter for the HB2A_Uplink hardware transceiver."""
from pathlib import Path
import queue
import sys
import threading
import time

HOST = Path(__file__).resolve().parents[1] / "HB2A_Uplink" / "Host"
if str(HOST) not in sys.path:
    sys.path.insert(0, str(HOST))
from protocol import command_data, usb_command

try:
    import serial
except ImportError:
    serial = None
    serial_site = HOST / ".venv" / "Lib" / "site-packages"
    if serial_site.exists():
        sys.path.insert(0, str(serial_site))
        try:
            import serial
        except ImportError:
            serial = None


def choose_hardware_port(ports, preferred=""):
    """Keep an explicit choice; only auto-select one plausible HB2A device."""
    if any(p.device == preferred for p in ports):
        return preferred
    candidates = [p.device for p in ports if
                  (getattr(p, "vid", None), getattr(p, "pid", None)) == (0x0483, 0x5740) or
                  any(name in (p.description or "").upper() for name in ("HB2A", "USTC TELEMETRY"))]
    return candidates[0] if len(candidates) == 1 else ""


class HardwareTransmitter:
    """Non-blocking USB-C serial transmitter using the board's own RF modem."""

    def __init__(self, report, port, power=10, recorder=None):
        self.report, self.port_name, self.power, self.recorder = report, port, int(power), recorder
        if not port:
            raise ValueError("Select the hardware transceiver COM port")
        if not 2 <= self.power <= 17:
            raise ValueError("Hardware transceiver power must be 2..17 dBm")
        if serial is None:
            raise RuntimeError("pyserial is required for the hardware transceiver")
        self.queue, self.armed, self.stop_event = queue.Queue(8), False, threading.Event()
        self.port = None
        self.worker = threading.Thread(target=self.run, name="HB2A hardware TX", daemon=True)
        self.worker.start()

    def _exchange(self, payload):
        port = self.port
        port.write(payload)
        response = bytearray()
        expected = "OK HB2A-UPLINK/1 " if payload == b"STATUS\n" else "OK SENT "
        echoes = ("", "^X", "\x18", payload.decode("ascii").strip())
        last_line = ""
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            if self.stop_event.is_set():
                raise RuntimeError("Hardware connection cancelled")
            chunk = port.read(1)
            if chunk == b"\n":
                last_line = response.decode("ascii", errors="replace").strip()
                response.clear()
                if last_line in echoes:
                    continue
                if last_line.startswith(expected) or last_line.startswith("ERR "):
                    return last_line
                raise RuntimeError("Unexpected reply on %s: %r; check Hardware COM and HB2A-UPLINK firmware" %
                                   (self.port_name, last_line))
            if not chunk:
                continue
            response.extend(chunk)
            if len(response) > 512:
                raise ValueError("Invalid hardware transceiver response on " + self.port_name)
        raise TimeoutError("No valid %s reply on %s (last line %r, partial %r)" %
                           (payload.split()[0].decode("ascii"), self.port_name, last_line, bytes(response)))

    def _open(self):
        if self.recorder:
            self.recorder.event("hardware_tx_opening", port=self.port_name)
        self.port = serial.Serial(self.port_name, 115200, timeout=0.1, write_timeout=1)
        self.port.reset_input_buffer()
        self.port.write(b"\x18\n")
        if self.stop_event.wait(.1):
            raise RuntimeError("Hardware connection cancelled")
        self.port.reset_input_buffer()
        reply = self._exchange(b"STATUS\n")
        if not reply.startswith("OK HB2A-UPLINK/1 ") or "RADIO=READY" not in reply.split():
            raise RuntimeError("Hardware transceiver on %s is not ready: %r" % (self.port_name, reply))
        if self.recorder:
            self.recorder.event("hardware_tx_connected", port=self.port_name, status=reply)

    def send(self, command, fields=()):
        if not self.armed:
            raise RuntimeError("Hardware transmitter is not armed")
        payload = usb_command(command, fields, self.power)
        # Validate and retain the exact application bytes in the ground log.
        data = command_data(command, fields)
        self.queue.put_nowait((command, list(fields), payload, data))

    def disarm(self):
        self.armed = False
        while True:
            try:
                self.queue.get_nowait()
            except queue.Empty:
                return

    def run(self):
        while not self.stop_event.is_set():
            try:
                command, fields, payload, data = self.queue.get(timeout=.1)
            except queue.Empty:
                continue
            if not self.armed:
                continue
            try:
                if self.port is None:
                    self._open()
                if not self.armed or self.stop_event.is_set():
                    continue
                reply = self._exchange(payload)
                if self.recorder:
                    self.recorder.event("hardware_tx", command=command, fields=fields,
                                        power_dbm=self.power, data_hex=data.hex().upper(), reply=reply,
                                        port=self.port_name)
                if reply.startswith("OK SENT "):
                    self.report({"tx": command, "hardware": True, "reply": reply})
                else:
                    raise RuntimeError(reply or "Hardware transceiver rejected command")
            except Exception as exc:
                if self.port is not None:
                    try:
                        self.port.close()
                    finally:
                        self.port = None
                self.disarm()
                error = "%s: %s" % (self.port_name, exc)
                if self.recorder:
                    self.recorder.event("hardware_tx_error", port=self.port_name, command=command, error=error)
                self.report({"error": error, "hardware": True})

    def close(self):
        self.disarm()
        self.stop_event.set()
        if self.port is not None:
            self.port.close()
            self.port = None
        self.worker.join(timeout=4)
        if self.worker.is_alive():
            raise RuntimeError("Hardware transmitter did not stop")
