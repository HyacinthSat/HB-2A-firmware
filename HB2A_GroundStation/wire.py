"""Project wire adaptation around the exact flight C codecs."""
import bootstrap  # noqa: F401
import ctypes as ct
import importlib.util
import os
from pathlib import Path
import zlib

import numpy as np

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("hb2a_commands", HERE.parent / "HB2A_Uplink/Host/protocol.py")
commands = importlib.util.module_from_spec(spec)
spec.loader.exec_module(commands)
RATES = (4800, 9600, 19200, 38400, 57600, 115200)
ACTUAL_RATES = tuple(32000000 / round(32000000 / r) for r in RATES)
DEPTHS = (1, 2, 3, 4, 5, 8)
PUNCTURES = ("1", "11", "1101", "110110")
ASM = bytes.fromhex("1acffc1d")


def pn17():
    state = 0x18E38
    out = bytearray(255)
    for i in range(255):
        for _ in range(8):
            bit = state & 1
            state = (state >> 1) ^ (0x12000 if bit else 0)
            out[i] = (out[i] << 1 | bit) & 255
    return np.frombuffer(bytes(out), dtype=np.uint8)


PN17 = pn17()


def coded_length(input_bits, code):
    if code == 0:
        return input_bits
    pattern = PUNCTURES[code]
    pairs = (input_bits + 6) * 2
    return pairs // len(pattern) * pattern.count("1") + pattern[:pairs % len(pattern)].count("1")


class Wire:
    def __init__(self):
        path = HERE / "build" / ("hb2a_wire.dll" if os.name == "nt" else "libhb2a_wire.so")
        self.lib = ct.CDLL(str(path))
        ptr = ct.c_void_p
        self.lib.uplink_frame_encode.argtypes = [ptr, ct.c_size_t, ptr, ct.c_size_t]
        self.lib.gmsk_high_speed_encode.argtypes = [ptr, ct.c_size_t, ct.c_uint8, ct.c_uint8,
                                                  ptr, ct.c_size_t, ct.POINTER(ct.c_size_t)]
        self.lib.gmsk_downlink_encode.argtypes = [ptr, ct.c_size_t, ptr, ct.c_size_t, ct.POINTER(ct.c_size_t)]
        self.lib.decode_rs_8.argtypes = [ptr, ptr, ct.c_int, ct.c_int]
        self.lib.encode_rs_8.argtypes = [ptr, ptr, ct.c_int]
        self.lib.ground_jpeg.argtypes = [ptr, ct.c_size_t, ptr, ct.c_size_t, ct.POINTER(ct.c_size_t)]

    def uplink(self, command, fields=()):
        data = commands.command_data(command, list(fields))
        out = ct.create_string_buffer(80)
        size = self.lib.uplink_frame_encode(data, len(data), out, len(out))
        if size < 0:
            raise ValueError("Uplink encoding failed")
        # Extra preamble for radio key-up, compatible with the FIFO receiver.
        return np.unpackbits(np.frombuffer(b"\x55" * 16 + out.raw[:size], np.uint8))

    def downlink(self, frames, code=1, depth=1):
        if code not in range(4) or depth not in DEPTHS or not 1 <= len(frames) <= depth:
            raise ValueError("Invalid coding profile")
        if any(len(f) != 223 for f in frames):
            raise ValueError("Each TM frame must have 223 bytes")
        data = b"".join(frames)
        out, bits = ct.create_string_buffer(8192), ct.c_size_t()
        rc = self.lib.gmsk_high_speed_encode(data, len(frames), code, depth, out, len(out), ct.byref(bits))
        if rc:
            raise ValueError("Downlink encoding failed: %d" % rc)
        return np.unpackbits(np.frombuffer(out.raw, np.uint8))[:bits.value]

    def rs_decode(self, codeword):
        buf = ct.create_string_buffer(bytes(codeword), 255)
        corrected = self.lib.decode_rs_8(buf, None, 0, 0)
        if corrected < 0:
            raise ValueError("RS uncorrectable")
        return buf.raw[:223], corrected

    def rs_encode(self, data):
        if len(data) != 223:
            raise ValueError("RS data must have 223 bytes")
        parity = ct.create_string_buffer(32)
        self.lib.encode_rs_8(data, parity, 0)
        return data + parity.raw

    def jpeg(self, packets):
        out, size = ct.create_string_buffer(4 * 1024 * 1024), ct.c_size_t()
        rc = self.lib.ground_jpeg(b"".join(packets), len(packets), out, len(out), ct.byref(size))
        if rc != 0:
            raise ValueError("SSDV decode failed: %d" % rc)
        return out.raw[:size.value]


def text_frame(text):
    return b"\x00\x02\x00\x00" + text.encode("ascii").ljust(219, b" ")


def parse_frame(data, callsign="BG7ZDQ"):
    if len(data) != 223 or data[:2] not in (b"\x00\x02", b"\x00\x04"):
        raise ValueError("Unexpected TM header")
    if data[1] == 2:
        return {"type": "text", "text": data[4:].decode("ascii").rstrip(" ")}
    expected = zlib.crc32(callsign.encode("ascii") + data[4:219])
    if expected != int.from_bytes(data[219:], "big"):
        raise ValueError("SSDV CRC mismatch (check callsign)")
    return {"type": "image", "image": data[4], "packet": int.from_bytes(data[5:7], "big"),
            "eoi": bool(data[10] & 4), "data": data}


def ssdv_packet(frame, callsign, wire):
    parse_frame(frame, callsign)
    call = 0
    for char in reversed(callsign):
        call = call * 40 + (ord(char) - 65 + 14 if char.isalpha() else int(char) + 1)
    dims = int.from_bytes(frame[7:10], "big")
    packet = b"\x55\x66" + call.to_bytes(4, "big") + frame[4:7]
    packet += bytes([dims >> 12, dims & 0xff]) + frame[10:14] + frame[14:219]
    packet += zlib.crc32(packet[1:]).to_bytes(4, "big")
    return packet[:1] + wire.rs_encode(packet[1:224])
