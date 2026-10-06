"""Image lifecycle checks using real Tk widgets, without radio hardware."""
from pathlib import Path
import sys
import tempfile
import tkinter as tk
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from PIL import Image
from station import App, Archive


class PreviewTests(unittest.TestCase):
    def setUp(self):
        self.root = tk.Tk()
        self.root.withdraw()
        self.app = App.__new__(App)
        self.app.preview = tk.Label(self.root)
        self.app.clear_preview()

    def tearDown(self):
        self.root.destroy()

    def test_new_image_removes_old_photo_and_replaces_pixels(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "image.png"
            Image.new("RGB", (32, 32), "red").save(path)
            self.app.show_image(path)
            old_photo = str(self.app.photo)
            self.app.image_progress(dict(frames=[dict(type="image", image=5, packet=3)]))
            self.assertEqual(str(self.app.preview.cget("image")), "")
            self.assertIsNone(self.app.photo)
            self.assertIn("Receiving image 5", self.app.preview.cget("text"))
            self.app.image_progress(dict(frames=[dict(type="image", image=5, packet=3)]))
            self.assertEqual(len(self.app.preview_packets), 1)
            Image.new("RGB", (32, 32), "blue").save(path)
            details = dict(image=5, packets=1, missing_packets=[0, 1, 2], eoi=False, complete=False)
            self.app.show_image(path, details)
            self.assertFalse(self.app.preview_complete)
            self.assertIn("no EOI yet", self.app.preview.cget("text"))
            self.assertNotEqual(str(self.app.photo), old_photo)
            self.assertEqual(self.root.tk.call(str(self.app.photo), "get", 0, 0), (0, 0, 255))
            self.app.show_image(path)
            # Identical packet retransmissions must not clear a completed image.
            self.app.preview_first_data = b"old"
            self.app.image_progress(dict(frames=[dict(type="image", image=5, packet=0, data=b"old")]))
            self.assertIsNotNone(self.app.photo)
            # Reuse of an image ID with different packet 0 starts a new image.
            self.app.image_progress(dict(frames=[dict(type="image", image=5, packet=0, data=b"new")]))
            self.assertIsNone(self.app.photo)
            self.assertEqual(self.app.preview_packets, {0})

    def test_business_start_clears_preview_and_old_packet_cache(self):
        self.app.next_job = tk.StringVar(value="REQF")
        self.app.file_index = tk.StringVar(value="2")
        self.app.business = SimpleNamespace(number=1)
        archive = Archive.__new__(Archive)
        archive.images, archive.generation = {5: {0: b"old"}}, 0
        archive.image_states = {}
        archive.recorder = Mock()
        self.app.archive = archive
        self.app.preview.configure(text="Old image")
        self.assertEqual(self.app.parameters(), ("REQF", ["2"]))
        self.assertEqual(archive.images, {})
        self.assertEqual(archive.generation, 1)
        self.assertEqual(self.app.preview.cget("text"), "Waiting for new image")


if __name__ == "__main__":
    unittest.main()
