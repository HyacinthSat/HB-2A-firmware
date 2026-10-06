import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import bootstrap  # noqa: F401
import numpy as np
from scipy import signal
from gnuradio import blocks, gr

from dsp import DecodeProfile, PacketSink, FrameDecoder, Receiver, estimate_esn0, simulated_iq, tx_baseband
from negotiation import Negotiator
from wire import ACTUAL_RATES, DEPTHS, RATES, Wire, text_frame


class WireTests(unittest.TestCase):
    def test_exhausted_candidates_release_slots_for_next_probe(self):
        bits = Wire().downlink([text_frame("HSTST,5,1,4")] * 4, 1, 4)[128:]
        received = []
        sink = PacketSink(5, 1, bits[:64],
                          SimpleNamespace(sample_rate=521000, measure=lambda *args: 10.),
                          received.append, "BG7ZDQ", profile=DecodeProfile())
        noise = np.zeros(17000 * 16, np.float32)
        try:
            with patch.object(sink, "nitems_read", return_value=0), patch.object(
                    sink, "get_tags_in_window", return_value=[SimpleNamespace(offset=i * 17000)
                                                             for i in range(16)]):
                sink.work([noise], [])
            self.assertEqual(sink.pending, [])
            self.assertEqual(len(sink.buffer), 0)
            payload = bits[64:].astype(np.float32) * 2 - 1
            with patch.object(sink, "nitems_read", return_value=len(noise)), patch.object(
                    sink, "get_tags_in_window", return_value=[SimpleNamespace(offset=len(noise))]):
                sink.work([payload], [])
            self.assertEqual(len(received), 1)
            self.assertEqual(received[0]["frames"][0]["text"], "HSTST,5,1,4")
        finally:
            if sink.decoder:
                sink.decoder.close()

    def test_repeated_hscon_keeps_partial_probe(self):
        soft = Wire().downlink([text_frame("HSTST,5,1,4")] * 4, 1, 4)[128:]
        profile, received = DecodeProfile(), []
        history = SimpleNamespace(sample_rate=521000, measure=lambda *args: 10.)
        sink = PacketSink(5, 1, soft[:64], history, received.append, "BG7ZDQ", profile=profile)
        payload = soft[64:].astype(np.float32) * 2 - 1
        split = len(payload) * 3 // 4 + 20
        try:
            with patch.object(sink, "nitems_read", return_value=0), patch.object(
                    sink, "get_tags_in_window", return_value=[SimpleNamespace(offset=0)]):
                sink.work([payload[:split]], [])
            self.assertFalse(received)
            self.assertTrue(sink.pending)
            profile.set(1, 4)
            with patch.object(sink, "nitems_read", return_value=split), patch.object(
                    sink, "get_tags_in_window", return_value=[]):
                sink.work([payload[split:]], [])
            self.assertEqual(len(received), 1)
            self.assertEqual(received[0]["depth"], 4)
            self.assertTrue(all(f["text"] == "HSTST,5,1,4" for f in received[0]["frames"]))
            generation = profile.snapshot()[2]
            profile.set(2, 4)
            self.assertEqual(profile.snapshot(), (2, 4, generation + 1))
        finally:
            if sink.decoder:
                sink.decoder.close()

    def test_recorded_hssup_soft_retry_preserves_original(self):
        soft = np.fromfile(Path(__file__).parent / "fixtures/hssup-impulsive.soft.f32", np.float32)
        original = soft.copy()
        decoder = FrameDecoder(robust_retry=False)
        try:
            with self.assertRaisesRegex(ValueError, "RS uncorrectable"):
                decoder.decode(soft, 1, 1)
            decoder.robust_retry = True
            frames = decoder.decode(soft, 1, 1)
            expected = bytes([0, 2, 5, 5]) + text_frame("HSSUP,3F,0F,3F")[4:]
            self.assertEqual(frames[0]["data"], expected)
            self.assertEqual(frames[0]["decode_method"], "clipped_soft_retry")
            np.testing.assert_array_equal(soft, original)
        finally:
            decoder.close()

    def test_soft_retry_does_not_accept_noise(self):
        decoder = FrameDecoder()
        try:
            soft = np.random.default_rng(712).normal(size=4156).astype(np.float32)
            with self.assertRaises(ValueError):
                decoder.decode(soft, 1, 1)
        finally:
            decoder.close()

    def test_flight_encoders_all_codes_and_depths(self):
        wire, decoder = Wire(), FrameDecoder()
        try:
            for code in range(4):
                for depth in DEPTHS:
                    with self.subTest(code=code, depth=depth):
                        frames = [text_frame("HSTST,5,%d,%d" % (code, depth)) for _ in range(depth)]
                        bits = wire.downlink(frames, code, depth)[128:]
                        decoded = decoder.decode(bits.astype(np.float32) * 2 - 1, code, depth)
                        self.assertEqual(len(decoded), depth)
                        self.assertTrue(all(f["text"].startswith("HSTST,5,") for f in decoded))
        finally:
            decoder.close()

    def test_rs_corrects_16_byte_errors(self):
        wire = Wire()
        data = bytes(range(223))
        damaged = bytearray(wire.rs_encode(data))
        for i in range(16):
            damaged[i * 13] ^= 157
        decoded, corrected = wire.rs_decode(damaged)
        self.assertEqual(decoded, data)
        self.assertEqual(corrected, 16)

    def test_tx_is_real_shaped_baseband(self):
        values = tx_baseband(Wire().uplink("EXPR"))
        self.assertEqual(values.dtype, np.float32)
        self.assertGreater(len(values), 3000)
        self.assertLessEqual(float(np.max(np.abs(values))), .251)


class NegotiationTests(unittest.TestCase):
    def setup_session(self, threshold=3.5):
        sent = []
        n = Negotiator(lambda c, f: sent.append((c, f)), lambda _: None, threshold)
        n.start("REQF", ["0"], 0)
        n.text("HSSUP,3F,0F,3F", 0, 1, 1, 20, 1, 1)
        n.text("HSCON,5,1,1", 0, 1, 1, 20, 2, 2)
        return n, sent

    def test_threshold_boundary_for_every_rate(self):
        for threshold in (3.0, 3.25, 3.5):
            for rate in range(6):
                n, sent = self.setup_session(threshold)
                now = 2.0
                for i in reversed(range(rate + 1, 6)):
                    now += .1
                    n.text("HSTST,%d,1,1" % i, i, 1, 1, threshold - .01, now, now)
                    now += .1
                    n.text("HSCON,%d,1,1" % (i - 1), 0, 1, 1, 20, now, now)
                now += .1
                n.text("HSTST,%d,1,1" % rate, rate, 1, 1, threshold, now, now)
                self.assertEqual(sent[-1], ("CFRM", []))
                self.assertEqual(n.state, "transfer")

    def test_wrong_stale_and_invalid_probes_cannot_confirm(self):
        for esn0, rate, code, stamp in [(30, 4, 1, 3), (30, 5, 0, 3),
                                      (30, 5, 1, 1), (math.nan, 5, 1, 3), (None, 5, 1, 3)]:
            n, sent = self.setup_session()
            n.text("HSTST,5,1,1", rate, code, 1, esn0, 2.5, stamp)
            self.assertNotEqual(sent[-1][0], "CFRM")

    def test_missing_probe_falls_back(self):
        n, sent = self.setup_session()
        self.assertEqual(n.rate_status[5], "Testing")
        self.assertEqual(n.rate_status[4], "Not tested yet")
        n.tick(4)
        self.assertEqual(sent[-1], ("CHPRAM", ["4", "1", "1"]))
        self.assertEqual(n.rate_status[5], "No qualified probe")
        self.assertEqual(n.rate_status[4], "Awaiting HSCON")

    def test_lost_confirm_waits_until_repeat_probe_ends(self):
        n, sent = self.setup_session()
        n.text("HSTST,5,1,1", 5, 1, 1, 3.5, 2.2, 2.2)
        count = len(sent)
        n.text("HSCON,5,1,1", 0, 1, 1, 10, 7, 7)
        self.assertEqual(len(sent), count)
        self.assertEqual(n.state, "probe")
        n.text("HSTST,5,1,1", 5, 1, 1, 3.5, 7.2, 7.2)
        self.assertEqual(len(sent), count + 1)
        self.assertEqual(sent[-1][0], "CFRM")

    def test_transfer_rejection_result_code_is_logged(self):
        messages = []
        n = Negotiator(lambda *_: None, messages.append)
        n.start("REQF", ["0"], 0)
        n.text("HSREJ,TRANSFER,-6", 0, 1, 1, 20, 1, 1)
        self.assertEqual(n.state, "idle")
        self.assertIn("result -6", messages[-1])


class DspTests(unittest.TestCase):
    def test_all_six_rates_in_one_iq_stream(self):
        wire = Wire()
        fs = 921600
        samples = []
        for i, rate in enumerate(RATES):
            bits = wire.downlink([text_frame("HSTST,%d,1,1" % i)])
            iq = simulated_iq(bits, 8)
            iq = signal.resample_poly(iq, fs // (8 * rate), 1)
            samples.extend((np.zeros(fs // 20, np.complex64), iq))
        samples.append(np.zeros(fs // 10, np.complex64))
        iq = np.concatenate(samples)
        received = []
        receiver = Receiver(blocks.vector_source_c(iq.tolist(), False), fs, received.append)
        try:
            receiver.run()
        finally:
            receiver.close()
        decoded = {group["rate"] for group in received}
        self.assertEqual(decoded, set(range(6)))

    def test_noisy_probe_measurement_uses_its_own_rate(self):
        wire, rng = Wire(), np.random.default_rng(310)
        fs = 921600
        samples = []
        for i, rate in enumerate(RATES):
            bits = wire.downlink([text_frame("HSTST,%d,1,1" % i)])
            iq = signal.resample_poly(simulated_iq(bits, 8), fs // (8 * rate), 1)
            for _ in range(3):
                segment = np.r_[np.zeros(fs // 20, np.complex64), iq]
                sigma = math.sqrt(fs / rate / 10 ** (12 / 10) / 2)
                noise = rng.normal(size=len(segment)) + 1j * rng.normal(size=len(segment))
                samples.append((segment + sigma * noise).astype(np.complex64))
        samples.append(np.zeros(fs // 10, np.complex64))
        received = []
        receiver = Receiver(blocks.vector_source_c(np.concatenate(samples).tolist(), False), fs, received.append)
        try:
            receiver.run()
        finally:
            receiver.close()
        measured = {g["rate"]: g["esn0"] for g in received if g["esn0"] is not None}
        self.assertEqual(set(measured), set(range(6)), [(g["rate"], g["esn0"]) for g in received])
        for value in measured.values():
            self.assertAlmostEqual(value, 12, delta=1.0)

    def test_esn0_awgn_all_rates(self):
        rng = np.random.default_rng(901)
        for rate in RATES:
            iq = simulated_iq(rng.integers(0, 2, 12000, dtype=np.uint8), 8)
            for target in (2.5, 3.0, 3.5, 5.0):
                noise_power = 8 / 10 ** (target / 10)
                noise = (rng.normal(size=len(iq)) + 1j * rng.normal(size=len(iq))) * math.sqrt(noise_power / 2)
                measured = estimate_esn0(iq + noise, 8 * rate, rate)
                self.assertAlmostEqual(measured, target, delta=.45)

    def test_gnuradio_iq_to_text(self):
        wire = Wire()
        rate = 5
        bits = wire.downlink([text_frame("HSTST,5,1,1")])
        iq = simulated_iq(bits, 8)
        # Leading/trailing silence also exercises acquisition after idle.
        iq = np.r_[np.zeros(4096, np.complex64), iq, np.zeros(12000, np.complex64)]
        received = []
        source = blocks.vector_source_c(iq.tolist(), False)
        receiver = Receiver(source, ACTUAL_RATES[rate] * 8, received.append)
        try:
            receiver.run()
        finally:
            receiver.close()
        frames = [f["text"] for group in received for f in group["frames"] if f["type"] == "text"]
        self.assertIn("HSTST,5,1,1", frames)


if __name__ == "__main__":
    unittest.main()
