import queue
import sys
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "Host"))
from uplink_gui import SerialWorker


class FakeSerial:
    def __init__(self, *args, **kwargs):
        self.writes = []
        self.pending = bytearray()
        self.closed = False

    def write(self, data):
        self.writes.append(data)
        if data == b"STATUS\n":
            self.pending.extend(b"OK HB2A-UPLINK/1 RADIO=READY\r\n")
        elif data.startswith(b"SEND"):
            self.pending.extend(b"OK SENT BYTES=30 DBM=10\r\n")
        else:
            self.pending.extend(b"ERR LINE_TOO_LONG_OR_OVERFLOW\r\n")
        return len(data)

    def read(self, n):
        result = bytes(self.pending[:n])
        del self.pending[:n]
        return result

    def reset_input_buffer(self):
        self.pending.clear()

    def close(self):
        self.closed = True


class WorkerTests(unittest.TestCase):
    def test_connect_does_not_transmit_and_explicit_send_runs_once(self):
        fake = FakeSerial()
        events = queue.Queue()
        worker = SerialWorker(events)
        with patch("uplink_gui.serial", SimpleNamespace(Serial=lambda *a, **kw: fake)):
            worker.start()
            worker.jobs.put(("open", "TEST"))
            self.assertEqual(events.get(timeout=2)[0], "connected")
            self.assertEqual(fake.writes, [b"\x18\n", b"STATUS\n"])
            command = b"SEND 10 45585052202020\n"
            worker.jobs.put(("send", command))
            self.assertEqual(events.get(timeout=2)[0], "sent")
            self.assertEqual(fake.writes.count(command), 1)
            worker.jobs.put(("quit", None))
            worker.join(timeout=2)
        self.assertTrue(fake.closed)
        self.assertFalse(worker.is_alive())

    def test_write_failure_disconnects_without_retry(self):
        fake = FakeSerial()
        events = queue.Queue()
        worker = SerialWorker(events)
        worker.port = fake
        with patch.object(fake, "write", side_effect=OSError("Disconnected")) as write:
            worker.start()
            worker.jobs.put(("send", b"SEND 10 45585052202020\n"))
            self.assertEqual(events.get(timeout=2), ("error", ("send", "Disconnected")))
            self.assertEqual(write.call_count, 1)
            worker.jobs.put(("quit", None))
            worker.join(timeout=2)
        self.assertTrue(fake.closed)
