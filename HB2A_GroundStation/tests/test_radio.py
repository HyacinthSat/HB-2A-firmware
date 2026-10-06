from pathlib import Path
import sys
import threading
import unittest
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import bootstrap
import radio


class PttTests(unittest.TestCase):
    def test_disconnected_output_never_keys_radio(self):
        results, calls, finished = [], [], threading.Event()
        class FakeRig:
            def check_frequency(self):
                pass
            def ptt(self, enabled):
                calls.append(enabled)
        def report(event):
            results.append(event)
            if "error" in event:
                finished.set()
        with patch.object(radio, "output_devices", return_value=[("1", "USB Audio CODEC")]), \
             patch.object(radio, "validate_output", side_effect=RuntimeError("Audio disconnected")), \
             patch.object(radio.audio, "sink") as sink, \
             patch.object(radio, "tx_baseband", return_value=[0.0]):
            tx = radio.Transmitter(report, device="1", rig=FakeRig())
            try:
                tx.armed = True
                tx.send("EXPR")
                self.assertTrue(finished.wait(3))
                self.assertNotIn(True, calls)
                sink.assert_not_called()
                self.assertFalse(tx.armed)
            finally:
                tx.close()

    def test_audio_failure_releases_ptt_and_disarms(self):
        results, calls, finished = [], [], threading.Event()
        class FakeRig:
            def check_frequency(self):
                pass
            def ptt(self, enabled):
                calls.append(enabled)
        class FailedAudioGraph:
            def connect(self, *args):
                pass
            def run(self):
                raise RuntimeError("Audio disconnected")
            def stop(self):
                pass
            def wait(self):
                pass
        def report(event):
            results.append(event)
            if "error" in event:
                finished.set()
        with patch.object(radio.gr, "top_block", return_value=FailedAudioGraph()), \
             patch.object(radio.audio, "sink", return_value=object()), \
             patch.object(radio, "output_devices", return_value=[("1", "USB Audio CODEC")]), \
             patch.object(radio, "validate_output"), \
             patch.object(radio, "tx_baseband", return_value=[0.0, .1, -.1]):
            tx = radio.Transmitter(report, device="1", rig=FakeRig())
            try:
                tx.armed = True
                tx.send("EXPR")
                self.assertTrue(finished.wait(3))
                self.assertEqual(calls[0], True)
                self.assertFalse(calls[-1])
                self.assertFalse(tx.armed)
                self.assertTrue(any("Audio disconnected" in r.get("error", "") for r in results))
            finally:
                tx.close()


if __name__ == "__main__":
    unittest.main()
