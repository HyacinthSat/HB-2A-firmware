import ctypes as ct
from pathlib import Path
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "Host"))
from protocol import command_data, usb_command


class Command(ct.Structure):
    _fields_ = [("block_count", ct.c_uint8), ("data", ct.c_uint8 * 56), ("crc", ct.c_uint16)]


class Decoder(ct.Structure):
    _fields_ = [("sync_shift", ct.c_uint32), ("preamble_level", ct.c_uint8),
                ("preamble_seen", ct.c_uint8), ("preamble_bits", ct.c_uint8),
                ("syncing", ct.c_uint8), ("collecting", ct.c_uint8),
                ("expected_blocks", ct.c_uint8), ("raw_count", ct.c_uint16),
                ("raw_bits", ct.c_uint8 * 528), ("ready", ct.c_uint8), ("command", Command)]


class ProtocolTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        build = ROOT / "Tests/build"
        build.mkdir(exist_ok=True)
        subprocess.run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-shared",
                        "-IFirmware/Core/Inc", "Firmware/Core/Src/uplink_protocol.c",
                        "Firmware/Core/Src/ground_command.c", "-o", "Tests/build/ground.dll"], cwd=ROOT, check=True)
        cls.ground = ct.CDLL(str(build / "ground.dll"))
        cls.ground.GroundCommand_Encode.argtypes = [ct.c_char_p, ct.POINTER(ct.c_uint8), ct.c_void_p, ct.c_size_t]
        cls.ground.uplink_frame_encode.argtypes = [ct.c_void_p, ct.c_size_t, ct.c_void_p, ct.c_size_t]
        # Compile the actual receiver source and header, not a stale copied decoder.
        receiver = ROOT.parent / "HB2A_Balloon/Target/Core"
        source = receiver / "Src/uplink_protocol.c"
        subprocess.run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-shared",
                        "-I" + str(receiver / "Inc"), str(source), "-o", "Tests/build/balloon.dll"],
                       cwd=ROOT, check=True)
        cls.balloon = ct.CDLL(str(build / "balloon.dll"))

    def decode(self, frame):
        decoder, command = Decoder(), Command()
        self.balloon.uplink_decoder_init(ct.byref(decoder))
        for byte in frame:
            for bit in range(7, -1, -1):
                self.balloon.uplink_decoder_push_bit(ct.byref(decoder), (byte >> bit) & 1)
        return self.balloon.uplink_decoder_take(ct.byref(decoder), ct.byref(command)), command

    def encode(self, wire, capacity=86):
        power, frame = ct.c_uint8(), (ct.c_uint8 * 86)()
        n = self.ground.GroundCommand_Encode(wire.rstrip(b"\n"), ct.byref(power), frame, capacity)
        return n, power.value, bytes(frame[:max(0, n)])

    def test_all_commands_round_trip_through_original_decoder(self):
        cases = [("TRANPD", ["BG7ZDQ", "APZDQ7", "APZDQ7", "HELLO"]),
                 ("EXPR", []), ("CAPTR", ["A", "640", "480", "7"]), ("REQF", ["1"]),
                 ("HSPAR", ["0", "0", "1"]), ("CHPRAM", ["0", "0", "1"]),
                 ("CFRM", []), ("CANCEL", [])]
        for kind, fields in cases:
            with self.subTest(kind=kind):
                n, power, frame = self.encode(usb_command(kind, fields, 17))
                expected = command_data(kind, fields)
                self.assertEqual(n, 22 + len(expected) // 7 * 8)
                self.assertEqual(power, 17)
                ok, command = self.decode(frame)
                self.assertEqual(ok, 1)
                self.assertEqual(bytes(command.data[:len(expected)]), expected)
                # Hardware extends the preamble for clock recovery; allow lost initial bits.
                air = b"\x55" * 16 + frame
                self.assertEqual(self.decode(air)[0], 1)
                self.assertEqual(self.decode(air[12:])[0], 1)

    def test_short_encoder_padding_crc(self):
        data = (ct.c_uint8 * 4)(*b"EXPR")
        frame = (ct.c_uint8 * 86)()
        n = self.ground.uplink_frame_encode(data, 4, frame, 86)
        ok, command = self.decode(bytes(frame[:n]))
        self.assertEqual(ok, 1)
        self.assertEqual(bytes(command.data[:7]), b"EXPR   ")

    def test_rejects_corrupt_crc(self):
        _, _, frame = self.encode(usb_command("EXPR", []))
        corrupt = frame[:-1] + bytes([frame[-1] ^ 1])
        self.assertEqual(self.decode(corrupt)[0], 0)

    def test_encoder_boundaries_and_rejection(self):
        valid = usb_command("EXPR", [])
        self.assertLess(self.encode(valid, 29)[0], 0)
        for line in (b"", b"SEND", b"SEND 1 45585052202020", b"SEND 18 45585052202020",
                     b"SEND 999999 45585052202020", b"SEND 10 ZZ", b"SEND 10 45585052",
                     b"SEND 10 45585052200020", b"SEND 10 554E4B4E4F574E", valid.rstrip() + b"00",
                     b"SEND 10 4853504152202020"):
            with self.subTest(line=line):
                self.assertLess(self.encode(line)[0], 0)
        wrong_path = command_data("TRANPD", ["BG7ZDQ", "APZDQ7", "APZDQ7", "HELLO"]).replace(b" APZDQ7 HELLO", b" APZDQ6 HELLO")
        self.assertEqual(self.encode(b"SEND 10 " + wrong_path.hex().encode())[0], -4)

    def test_host_field_validation(self):
        self.assertEqual(len(command_data("CAPTR", ["A", "400", "304", "5"])), 35)
        for fields in (["BG7ZDQ", "APZDQ7", "APZDQ7", "TOOLONG!"],
                       ["BG7ZDQ", "APZDQ7", "APZDQ7", "\u6d4b\u8bd5"],
                       ["TOOLONG", "APZDQ7", "APZDQ7", "HI"],
                       ["BAD-16", "APZDQ7", "APZDQ7", "HI"],
                       ["BG7ZDQ", "APZDQ7", "WRONG", "HI"]):
            with self.assertRaises(ValueError):
                usb_command("TRANPD", fields)
        for fields in (["A", "401", "304", "5"], ["A", "400", "303", "5"],
                       ["A", "0", "304", "5"]):
            with self.assertRaises(ValueError):
                usb_command("CAPTR", fields)


if __name__ == "__main__":
    unittest.main()
