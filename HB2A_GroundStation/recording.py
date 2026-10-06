"""Append-only session evidence with sample-indexed SigMF IQ recordings."""
from datetime import datetime, timezone, timedelta
import json
import hashlib
import platform
from pathlib import Path
import threading
import time
import wave

import numpy as np


class Recorder:
    def __init__(self, directory, config):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.config = config
        self.lock, self.sequence = threading.RLock(), 0
        self.business_id = 0
        self.started = datetime.now(timezone.utc)
        self.logfile = (self.directory / "events.jsonl").open("a", encoding="utf-8", buffering=1)
        self.event("session_start", config=config)
        source = Path(__file__).resolve().parent
        paths = list(source.glob("*.py")) + [source / "native.c"]
        core = source.parent / "HB2A_Balloon/Target/Core/Src"
        paths += [core / (name + ".c") for name in ("gmsk_downlink", "uplink_protocol", "rs8", "ssdv", "high_speed_ssdv")]
        hashes = {str(path.relative_to(source.parent)): hashlib.sha256(path.read_bytes()).hexdigest()
                  for path in paths}
        self.event("provenance", python=platform.python_version(), platform=platform.platform(),
                   numpy=np.__version__, source_sha256=hashes)

    def event(self, kind, **details):
        with self.lock:
            self.sequence += 1
            record = dict(sequence=self.sequence, utc=datetime.now(timezone.utc).isoformat(),
                          monotonic=time.monotonic(), business_id=self.business_id, event=kind, **details)
            self.logfile.write(json.dumps(record, ensure_ascii=True, allow_nan=False) + "\n")
            return self.sequence

    def save_tx(self, command, fields, bits, values, **details):
        with self.lock:
            seq = self.event("tx_generated", command=command, fields=fields, samples=len(values),
                             sample_rate=48000, valid_bits=len(bits), **details)
            stem = self.directory / ("tx-%06d-%s" % (seq, command))
            np.asarray(values, dtype="<f4").tofile(str(stem) + ".f32")
            np.packbits(bits).tofile(str(stem) + ".bits")
            with wave.open(str(stem) + ".wav", "wb") as wav:
                wav.setnchannels(1)
                wav.setsampwidth(2)
                wav.setframerate(48000)
                wav.writeframes((np.clip(values, -1, 1) * 32767).astype("<i2").tobytes())
            return seq

    def packet(self, group, soft):
        with self.lock:
            summary = {k: v for k, v in group.items() if k != "frames"}
            summary["frames"] = [{k: v for k, v in f.items() if k != "data"} for f in group.get("frames", [])]
            seq = self.event("rx_packet" if "frames" in group else "rx_decode_failure", **summary)
            stem = self.directory / ("rx-%06d" % seq)
            np.asarray(soft, dtype="<f4").tofile(str(stem) + ".soft.f32")
            if "frames" in group:
                with open(str(stem) + ".tm", "wb") as stream:
                    for frame in group["frames"]:
                        stream.write(frame["data"])

    def iq_metadata(self, filename, sample_start, sample_count):
        metadata = {"global": {"core:datatype": "cf32_le", "core:version": "1.2.5",
                                 "core:sample_rate": self.config["sample_rate"],
                                 "core:description": "HB2A raw SDR IQ, before tuning/filtering",
                                 "hb2a:session_sample_start": sample_start},
                    "captures": [{"core:sample_start": 0, "core:frequency": self.config["frequency"],
                                  "core:datetime": (self.started + timedelta(seconds=sample_start / self.config["sample_rate"])).isoformat()}],
                    "annotations": [{"core:sample_start": 0, "core:sample_count": sample_count}]}
        Path(str(filename).replace(".sigmf-data", ".sigmf-meta")).write_text(
            json.dumps(metadata, indent=2), encoding="utf-8")

    def close(self):
        self.event("session_stop")
        self.logfile.close()
