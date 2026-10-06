from pathlib import Path
import sys
import tempfile
import json
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import bootstrap
import numpy as np
from gnuradio import blocks, gr
from negotiation import BusinessLoop
from recording import Recorder
from dsp import IqRecorder


class BusinessTests(unittest.TestCase):
    def test_invalid_settings_wait_then_reacquire_without_losing_one_shot_request(self):
        sent, logs, valid, reads = [], [], [False], []
        def parameters():
            reads.append(True)
            if not valid[0]:
                raise ValueError("CAPTR size must be a preset")
            return "REQF", ["4"]
        loop = BusinessLoop(lambda c, f: sent.append((c, f)), logs.append, parameters)
        loop.start(0)
        ack = dict(rate=0, code=1, depth=1, esn0=10, received_at=1,
                   frames=[dict(type="text", text="ACK,EXPR")])
        loop.group(ack, 1)
        loop.tick(2)
        self.assertTrue(loop.enabled)
        self.assertEqual(loop.state, "parameters")
        self.assertEqual(loop.number, 0)
        loop.tick(3)
        loop.tick(400)  # Allow the balloon lease to expire while editing.
        self.assertEqual(sent, [("EXPR", [])])
        self.assertEqual(sum("Invalid business settings" in m for m in logs), 1)
        valid[0] = True
        loop.tick(401)
        self.assertEqual(sent[-1], ("EXPR", []))
        count = len(reads)
        loop.group(ack, 402)
        loop.tick(403)
        self.assertEqual(sent[-1], ("REQF", ["4"]))
        self.assertEqual(len(reads), count)
        self.assertEqual(loop.number, 1)

    def test_stop_while_settings_invalid_prevents_auto_resume(self):
        loop = BusinessLoop(lambda *_: None, lambda *_: None,
                            lambda: (_ for _ in ()).throw(ValueError("invalid")))
        loop.enabled, loop.state = True, "ready"
        loop.tick(1)
        self.assertEqual(loop.state, "parameters")
        loop.stop()
        loop.tick(999)
        self.assertEqual(loop.state, "stopped")

    def test_complete_image_does_not_cancel_when_hsdone_is_lost(self):
        sent = []
        loop = BusinessLoop(lambda c, f: sent.append((c, f)), lambda _: None,
                            lambda: ("CAPTR", ["A", "640", "480", "5"]))
        loop.start(0)
        loop.group(dict(rate=0, code=1, depth=1, esn0=10, received_at=1,
                        frames=[dict(type="text", text="ACK,EXPR")]), 1)
        loop.tick(2)
        loop.group(dict(rate=0, code=1, depth=1, esn0=10, received_at=3,
                        frames=[dict(type="text", text="HSSUP,3F,0F,3F")]), 3)
        loop.group(dict(rate=0, code=1, depth=1, esn0=10, received_at=4,
                        frames=[dict(type="text", text="HSCON,5,1,4")]), 4)
        loop.group(dict(rate=5, code=1, depth=4, esn0=10, received_at=5,
                        frames=[dict(type="text", text="HSTST,5,1,4")]), 5)
        loop.group(dict(rate=5, code=1, depth=4, esn0=10, received_at=6,
                        frames=[dict(type="image", eoi=True)]), 6)
        loop.tick(11)
        self.assertNotIn(("CANCEL", []), sent)
        self.assertEqual(loop.session.state, "idle")

    def test_continuous_capture_and_one_shot_file(self):
        sent, requests = [], [["CAPTR", ["A", "640", "480", "5"]]]
        def parameters():
            result = requests.pop(0) if requests else ["CAPTR", ["B", "320", "240", "3"]]
            return result
        loop = BusinessLoop(lambda c, f: sent.append((c, f)), lambda text: None, parameters)
        def text(value, now, rate=0, code=1, depth=1):
            loop.group(dict(rate=rate, code=code, depth=depth, esn0=3.5,
                            received_at=now, frames=[dict(type="text", text=value)]), now)
        loop.start(0)
        self.assertEqual(sent[-1], ("EXPR", []))
        text("ACK,EXPR", 1)
        loop.tick(2)
        self.assertEqual(sent[-1][0], "CAPTR")
        text("HSSUP,3F,0F,3F", 3)
        text("HSCON,5,1,4", 4)
        text("HSTST,5,1,4", 4.2, 5, 1, 4)
        self.assertEqual(sent[-1][0], "CFRM")
        count = len(sent)
        loop.tick(20)
        self.assertEqual(len(sent), count)
        requests.append(["REQF", ["2"]])
        text("HSDONE", 21)
        loop.tick(21)
        loop.tick(22)
        text("ACK,EXPR", 23)
        loop.tick(24)
        self.assertEqual(sent[-1], ("REQF", ["2"]))
        text("HSREJ,NOFILE", 25)
        loop.tick(25)
        loop.tick(26)
        text("ACK,EXPR", 27)
        loop.tick(28)
        self.assertEqual(sent[-1], ("CAPTR", ["B", "320", "240", "3"]))

    def test_acquisition_retries_are_bounded_then_resume(self):
        sent = []
        loop = BusinessLoop(lambda c, f: sent.append(c), lambda _: None, lambda: ("REQF", ["0"]))
        loop.start(0)
        for now in (5, 10, 15):
            loop.tick(now)
        self.assertEqual(sent, ["EXPR"] * 3)
        loop.tick(29)
        self.assertEqual(len(sent), 3)
        loop.tick(30)
        self.assertEqual(len(sent), 4)
        loop.stop()
        loop.tick(999)
        self.assertEqual(len(sent), 4)


class RecordingTests(unittest.TestCase):
    def test_raw_iq_and_waveforms_roundtrip(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = Recorder(directory, dict(sample_rate=100, frequency=144925000))
            iq = (np.arange(13000) + 1j * np.arange(13000)[::-1]).astype(np.complex64)
            graph, sink = gr.top_block(), IqRecorder(recorder, 100)
            graph.connect(blocks.vector_source_c(iq.tolist(), False), sink)
            graph.run()
            sink.finish_segment()
            recorder.save_tx("EXPR", [], np.array([1, 0, 1], np.uint8), np.array([0, .2, -.2], np.float32))
            recorder.close()
            paths = sorted(Path(directory).glob("*.sigmf-data"))
            self.assertEqual(len(paths), 3)
            actual = np.concatenate([np.fromfile(p, "<c8") for p in paths])
            np.testing.assert_array_equal(actual, iq)
            total = sum(json.loads(p.read_text())["annotations"][0]["core:sample_count"]
                        for p in Path(directory).glob("*.sigmf-meta"))
            self.assertEqual(total, len(iq))
            self.assertEqual(len(list(Path(directory).glob("tx-*.wav"))), 1)
            events = [json.loads(line) for line in (Path(directory) / "events.jsonl").read_text().splitlines()]
            self.assertEqual(events[-1]["event"], "session_stop")


if __name__ == "__main__":
    unittest.main()
