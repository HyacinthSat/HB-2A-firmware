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
        self.port.write(payload)
        response = bytearray()
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            chunk = self.port.read(1)
            if chunk == b"\n":
                return response.decode("ascii", errors="replace").strip()
            if not chunk:
                continue
            response.extend(chunk)
            if len(response) > 256:
                raise ValueError("Invalid hardware transceiver response")
        raise TimeoutError("Hardware transceiver response timed out")

    def _open(self):
        self.port = serial.Serial(self.port_name, 115200, timeout=0.1, write_timeout=1)
        self.port.reset_input_buffer()
        self.port.write(b"\x18\n")
        self.port.reset_input_buffer()
        reply = self._exchange(b"STATUS\n")
        if not reply.startswith("OK HB2A-UPLINK/1 ") or "RADIO=READY" not in reply:
            raise RuntimeError("Hardware transceiver is not ready: " + reply)
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
                self.report({"error": str(exc), "hardware": True})

    def close(self):
        self.disarm()
        self.stop_event.set()
        if self.port is not None:
            self.port.close()
            self.port = None
        self.worker.join(timeout=4)
        if self.worker.is_alive():
            raise RuntimeError("Hardware transmitter did not stop")
