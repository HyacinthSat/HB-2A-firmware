from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import hardware_radio


class HardwareRadioTests(unittest.TestCase):
    def test_uses_existing_uplink_wire_protocol(self):
        self.assertEqual(hardware_radio.command_data("EXPR", []), b"EXPR   ")
        self.assertTrue(hardware_radio.usb_command("EXPR", [], 10).startswith(b"SEND 10 "))

    def test_rejects_invalid_power_before_worker_starts(self):
        with self.assertRaisesRegex(ValueError, "2..17"):
            hardware_radio.HardwareTransmitter(lambda _: None, "COM9", 18)


if __name__ == "__main__":
    unittest.main()
