import ctypes as ct
import io
from pathlib import Path
import shutil
import subprocess
import sys
import unittest
import tempfile
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from wire import Wire, ssdv_packet, parse_frame
from station import Archive
from PIL import Image


class ImageTests(unittest.TestCase):
    def test_real_flight_jpeg_to_tm_and_back(self):
        root = Path(__file__).resolve().parents[2]
        suffix = ".dll" if sys.platform == "win32" else ".so"
        fixture_path = root / "HB2A_GroundStation/build" / ("fixture" + suffix)
        subprocess.run([shutil.which("gcc"), "-shared", "-fPIC", "-O2",
                        "-I" + str(root / "HB2A_Balloon/Target/Core/Inc"),
                        str(root / "HB2A_Balloon/Target/Core/Src/factory_image.c"),
                        "-o", str(fixture_path)], check=True)
        fixture, wire = ct.CDLL(str(fixture_path)), Wire()
        length = ct.c_uint.in_dll(fixture, "g_factory_jpeg_len").value
        jpeg = (ct.c_ubyte * length).in_dll(fixture, "g_factory_jpeg")
        codec = ct.create_string_buffer(wire.lib.ground_ssdv_size())
        packet, frame = ct.create_string_buffer(256), ct.create_string_buffer(223)
        for name in ("ssdv_enc_init", "ssdv_enc_feed", "ssdv_enc_get_packet", "ssdv_enc_set_buffer"):
            getattr(wire.lib, name).restype = ct.c_byte
        wire.lib.ssdv_enc_init.argtypes = [ct.c_void_p, ct.c_uint8, ct.c_char_p, ct.c_uint8, ct.c_int8, ct.c_int]
        wire.lib.ssdv_enc_feed.argtypes = [ct.c_void_p, ct.c_void_p, ct.c_size_t]
        wire.lib.high_speed_ssdv_build_frame.argtypes = [ct.c_void_p, ct.c_size_t, ct.c_char_p, ct.c_void_p, ct.c_size_t]
        self.assertEqual(wire.lib.ssdv_enc_init(codec, 0, b"BG7ZDQ", 3, 2, 256), 0)
        self.assertEqual(wire.lib.ssdv_enc_set_buffer(codec, packet), 0)
        self.assertEqual(wire.lib.ssdv_enc_feed(codec, jpeg, length), 0)
        packets, frames = [], []
        for _ in range(1000):
            result = wire.lib.ssdv_enc_get_packet(codec)
            if result == 4:
                break
            self.assertEqual(result, 0)
            self.assertEqual(wire.lib.high_speed_ssdv_build_frame(packet, 256, b"BG7ZDQ", frame, 223), 0)
            restored = ssdv_packet(frame.raw, "BG7ZDQ", wire)
            self.assertEqual(restored, packet.raw)
            packets.append(restored)
            frames.append(parse_frame(frame.raw))
            self.assertEqual(wire.lib.ssdv_enc_set_buffer(codec, packet), 0)
        else:
            self.fail("SSDV did not finish")
        output = wire.jpeg(packets)
        self.assertTrue(output.startswith(b"\xff\xd8"))
        self.assertTrue(output.endswith(b"\xff\xd9"))
        (root / "HB2A_GroundStation/build/roundtrip.jpg").write_bytes(output)
        self.assertGreater(len(frames), 12)
        with tempfile.TemporaryDirectory() as directory, patch("station.time.monotonic", return_value=100):
            archive = Archive(directory, "BG7ZDQ", {})
            try:
                paths = archive.save(dict(frames=frames[:4]))
                self.assertEqual(len(paths), 1)
                partial = Path(paths[0])
                initial = partial.read_bytes()
                with Image.open(partial) as preview:
                    preview.load()
                self.assertFalse(archive.updates[str(partial)]["eoi"])
                self.assertEqual(archive.save(dict(frames=frames[4:8])), [])
                archive.flush(force=True)
                self.assertNotEqual(partial.read_bytes(), initial)
                archive.save(dict(frames=frames[9:]))  # Lose packet 8, but receive EOI.
                info = archive.updates[str(partial)]
                self.assertTrue(info["eoi"])
                self.assertFalse(info["complete"])
                self.assertEqual(info["missing_packets"], [8])
                with Image.open(partial) as preview:
                    preview.load()
                archive.save(dict(frames=[frames[8]]))  # Late packet fills the gap.
                final = archive.flush(force=True)[0]
                self.assertTrue(archive.updates[final]["complete"])
                self.assertEqual(Path(final).read_bytes(), output)
                self.assertEqual(archive.save(dict(frames=[frames[0]])), [])
                archive.begin_business()
                # Missing first and last packets still yields a displayable snapshot.
                path = archive.save(dict(frames=frames[1:4]))[0]
                with Image.open(path) as preview:
                    preview.load()
                archive.save(dict(frames=frames[4:7]))
                archive.begin_business()  # No EOI: flush newest partial before clearing.
                self.assertEqual(archive.updates[path]["packets"], 6)
                self.assertEqual(archive.updates[path]["missing_packets"], [0])
            finally:
                archive.close()


if __name__ == "__main__":
    unittest.main()
