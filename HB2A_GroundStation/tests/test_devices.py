from pathlib import Path
import sys
import unittest
from unittest.mock import MagicMock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from station import App, make_source, parser, validate_capture_dimensions
from audio_devices import resolve_output, validate_output


class DeviceTests(unittest.TestCase):
    def test_pluto_config_and_readback(self):
        args = parser().parse_args([])
        osmosdr, iio = MagicMock(), MagicMock()
        source = osmosdr.source.return_value
        source.set_sample_rate.return_value = source.get_sample_rate.return_value = 521000
        source.get_bandwidth.return_value = 750000
        with patch.dict(sys.modules, osmosdr=osmosdr, iio=iio):
            self.assertIs(make_source(args), source)
        osmosdr.source.assert_called_once_with("soapy=0,driver=plutosdr,uri=ip:192.168.1.10")
        source.set_gain_mode.assert_called_once_with(True)
        source.set_gain.assert_not_called()
        source.set_bandwidth.assert_called_once_with(400000)
        iio.Context.assert_not_called()
        self.assertEqual(args.rx_device["agc"], "driver automatic")
        self.assertEqual(args.rx_device["sample_rate"], 521000)

    def test_wrong_sample_rate_rejected(self):
        osmosdr = MagicMock()
        osmosdr.source.return_value.set_sample_rate.return_value = 1000000
        with patch.dict(sys.modules, osmosdr=osmosdr):
            with self.assertRaisesRegex(ValueError, "sample rate"):
                make_source(parser().parse_args([]))

    def test_outgoing_profile_uses_code_and_interleave_fields(self):
        app = object.__new__(App)
        app.receiver = MagicMock()
        app.transmitter = MagicMock()
        app.level = MagicMock()
        app.level.get.return_value = .25
        app.invert = MagicMock()
        app.invert.get.return_value = False
        app.log = MagicMock()
        app.send("HSPAR", ["5", "1", "4"])
        app.receiver.set_profile.assert_called_once_with(1, 4)

    def test_camera_accepts_presets_only(self):
        for size in ((128, 96), (256, 192), (320, 240), (480, 272), (640, 480), (800, 480)):
            self.assertEqual(validate_capture_dimensions(*map(str, size)), size)
        for size in ((1024, 768), (1280, 960), (640, 0), (641, 480), (160, 120)):
            with self.assertRaises(ValueError):
                validate_capture_dimensions(*size)

    def test_audio_selection_requires_unique_output(self):
        devices = [("0", "Speakers"), ("1", "USB Audio CODEC"), ("2", "USB Audio CODEC")]
        self.assertEqual(resolve_output("2", devices), devices[2])
        for value in ("", "Missing", "USB Audio CODEC"):
            with self.assertRaises(ValueError):
                resolve_output(value, devices)

    def test_unplug_or_device_renumbering_rejected(self):
        with patch("audio_devices.output_devices", return_value=[("1", "Speakers")]):
            with self.assertRaisesRegex(RuntimeError, "disconnected"):
                validate_output("1", "USB Audio CODEC")


if __name__ == "__main__":
    unittest.main()
