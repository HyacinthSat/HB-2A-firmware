from pathlib import Path
import sys
import unittest
import threading
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import hardware_radio


class FakePort:
    def __init__(self, status=b"OK HB2A-UPLINK/1 RADIO=READY\r\n", sent=b"OK SENT BYTES=70\r\n"):
        self.status, self.sent, self.input = status, sent, bytearray()
        self.writes = []

    def write(self, payload):
        self.writes.append(payload)
        self.input.extend(self.status if payload == b"STATUS\n" else
                          self.sent if payload.startswith(b"SEND ") else b"^X\r\n")

    def read(self, count):
        chunk = bytes(self.input[:count])
        del self.input[:count]
        return chunk

    def reset_input_buffer(self):
        self.input.clear()

    def close(self):
        pass


class HardwareRadioTests(unittest.TestCase):
    def adapter(self, port):
        tx = hardware_radio.HardwareTransmitter.__new__(hardware_radio.HardwareTransmitter)
        tx.port, tx.port_name, tx.recorder = port, "COM10", None
        tx.stop_event = threading.Event()
        return tx

    def test_default_port_uses_usb_identity_not_first_com(self):
        ch340 = SimpleNamespace(device="COM7", description="USB-SERIAL CH340", vid=0x1A86, pid=0x7523)
        stm32 = SimpleNamespace(device="COM10", description="USB Serial Device", vid=0x0483, pid=0x5740)
        self.assertEqual(hardware_radio.choose_hardware_port([ch340, stm32]), "COM10")
        self.assertEqual(hardware_radio.choose_hardware_port([ch340]), "")
        self.assertEqual(hardware_radio.choose_hardware_port([ch340, stm32], "COM7"), "COM7")
        second = SimpleNamespace(device="COM11", description="USB Serial Device", vid=0x0483, pid=0x5740)
        self.assertEqual(hardware_radio.choose_hardware_port([stm32, second]), "")

    def test_status_skips_empty_lines_and_clear_command_echo(self):
        port = FakePort(b"\r\n^X\r\n\x18\nSTATUS\r\nOK HB2A-UPLINK/1 RADIO=READY\r\n")
        reply = self.adapter(port)._exchange(b"STATUS\n")
        self.assertEqual(reply, "OK HB2A-UPLINK/1 RADIO=READY")
        self.assertEqual(port.writes, [b"STATUS\n"])

    def test_unrelated_console_is_not_accepted(self):
        with self.assertRaisesRegex(RuntimeError, "COM10.*check Hardware COM"):
            self.adapter(FakePort(b"login:\r\n"))._exchange(b"STATUS\n")

    def test_timeout_names_port_and_command(self):
        tx = self.adapter(FakePort(b""))
        with patch("hardware_radio.time.monotonic", side_effect=[0, 0, 4]):
            with self.assertRaisesRegex(TimeoutError, "No valid STATUS reply on COM10"):
                tx._exchange(b"STATUS\n")

    def test_handshake_failure_does_not_send_rf_command(self):
        port, finished, reports = FakePort(b"OK HB2A-UPLINK/1 RADIO=ERROR\n"), threading.Event(), []
        def report(value):
            reports.append(value)
            finished.set()
        with patch.object(hardware_radio.serial, "Serial", return_value=port):
            tx = hardware_radio.HardwareTransmitter(report, "COM10")
            try:
                tx.armed = True
                tx.send("EXPR", [])
                self.assertTrue(finished.wait(2))
                self.assertIn("COM10", reports[0]["error"])
                self.assertFalse(tx.armed)
                self.assertFalse(any(p.startswith(b"SEND ") for p in port.writes))
            finally:
                tx.close()

    def test_send_echo_is_skipped_without_retransmitting(self):
        payload = hardware_radio.usb_command("EXPR", [], 10)
        port = FakePort(sent=payload + b"\nOK SENT BYTES=70\n")
        self.assertEqual(self.adapter(port)._exchange(payload), "OK SENT BYTES=70")
        self.assertEqual(port.writes, [payload])

    def test_uses_existing_uplink_wire_protocol(self):
        self.assertEqual(hardware_radio.command_data("EXPR", []), b"EXPR   ")
        self.assertTrue(hardware_radio.usb_command("EXPR", [], 10).startswith(b"SEND 10 "))

    def test_rejects_invalid_power_before_worker_starts(self):
        with self.assertRaisesRegex(ValueError, "2..17"):
            hardware_radio.HardwareTransmitter(lambda _: None, "COM9", 18)


if __name__ == "__main__":
    unittest.main()
