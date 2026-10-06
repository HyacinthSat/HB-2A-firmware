from pathlib import Path
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[1]


class FirmwareTests(unittest.TestCase):
    def build_run(self, name, sources):
        (ROOT / "Tests/build").mkdir(exist_ok=True)
        output = f"Tests/build/{name}.exe"
        subprocess.run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-ITests/fakes",
                        "-IFirmware/Core/Inc", *sources, "-o", output], cwd=ROOT, check=True)
        subprocess.run([str(ROOT / output)], cwd=ROOT, check=True)

    def test_radio(self):
        self.build_run("radio", ["Tests/test_radio.c", "Firmware/Core/Src/uplink_radio.c",
                                 "Firmware/Core/Src/uplink_protocol.c"])

    def test_usb(self):
        self.build_run("usb", ["Tests/test_usb.c", "Firmware/Core/Src/ground_station.c",
                               "Firmware/Core/Src/ground_command.c", "Firmware/Core/Src/uplink_protocol.c"])
