"""Replay one recorded IQ segment through the control decoder, without RF TX."""
from pathlib import Path
import argparse
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import bootstrap
import numpy as np
from scipy import signal
from gnuradio import blocks, digital, filter, gr
from satellites.components.demodulators import fsk_demodulator
from dsp import PacketSink, DecodeProfile, FrameDecoder
from wire import ACTUAL_RATES, RATES, Wire, text_frame, PN17


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("directory", type=Path)
    parser.add_argument("--clock-bw", type=float, default=.03)
    parser.add_argument("--clock-limit", type=float, default=.002)
    parser.add_argument("--offset", type=float, default=0)
    parser.add_argument("--soft", action="store_true")
    parser.add_argument("--inspect", action="store_true")
    parser.add_argument("--legacy-fec", action="store_true")
    parser.add_argument("--rate-id", type=int, choices=range(6), default=0)
    args = parser.parse_args()
    if args.inspect:
        iq = np.memmap(args.directory / "iq-000000000000.sigmf-data", dtype=np.complex64, mode="r")
        for second in (26.5, 27.5, 28.8, 29.8, 33.5, 34.5, 38.8, 42.8, 43.8):
            x = iq[int(second * 520999):int((second + .2) * 520999)]
            f, p = signal.welch(x, 520999, nperseg=16384, return_onesided=False)
            band = abs(f) < 10000
            print(second, "peak", f[np.argmax(p)], "centroid", np.sum(f[band]*p[band])/sum(p[band]),
                  "IQ", np.percentile(abs(x), [1, 50, 99]), flush=True)
        return
    if args.soft:
        decoder = FrameDecoder(robust_retry=False)
        try:
            for path in sorted(args.directory.glob("rx-*.soft.f32")):
                soft = np.fromfile(path, np.float32)
                if len(soft) != 4156:
                    continue
                raw = decoder.fec.decode(soft, 1, 2072)
                data = np.frombuffer(raw[4:], np.uint8) ^ PN17
                print(path.name, np.percentile(soft, [0, 10, 50, 90, 100]),
                      bytes(data[:40]), flush=True)
                for scale in (.25, .5, 1., 2.):
                    try:
                        frames = decoder.decode(soft * scale, 1, 1)
                        print("scale", scale, [f.get("text") for f in frames], flush=True)
                    except ValueError:
                        pass
                seq = int(path.name.split('-')[1].split('.')[0])
                known = {20:(0,'ACK,EXPR'),28:(2,'ACK,CAPTR'),36:(4,'ACK,CAPTR'),
                         38:(5,'HSSUP,3F,0F,3F'),48:(7,'HSSUP,3F,0F,3F'),
                         61:(8,'ACK,CANCEL'),64:(9,'ACK,EXPR'),71:(10,'ACK,EXPR')}
                if seq in known:
                    count, text = known[seq]
                    frame = bytes([0,2,count,count]) + text_frame(text)[4:]
                    expected = Wire().downlink([frame])[128:]
                    for delta in (-1,0,1):
                        a = soft[64+max(delta,0):len(soft)+min(delta,0)]
                        b = expected[64+max(-delta,0):len(soft)-max(delta,0)]
                        errors = np.flatnonzero((a>0) != b)
                        print('bit errors delta',delta,len(errors), errors[:25],flush=True)
                    for bound in (.5,1.,1.5):
                        try:
                            frames = decoder.decode(np.clip(soft,-bound,bound),1,1)
                            print('clip',bound,[f.get('text') for f in frames],flush=True)
                        except ValueError:
                            pass
        finally:
            decoder.close()
        return
    fs, rate = 520999., ACTUAL_RATES[args.rate_id]
    class History:
        sample_rate = fs
        def measure(self, *args):
            return None
    received = []
    def result(group):
        for frame in group["frames"]:
            if frame["type"] == "text":
                print(round(group["received_at"], 3), frame["text"],
                      frame.get("decode_method"), flush=True)
            received.append(frame)
    graph = gr.top_block()
    source = blocks.file_source(gr.sizeof_gr_complex,
                                str(args.directory / "iq-000000000000.sigmf-data"), False)
    rotator = blocks.rotator_cc(-2 * np.pi * args.offset / fs)
    decimation = int(fs / (4 * rate))
    channel = filter.fir_filter_ccc(decimation, filter.firdes.low_pass(1, fs, .75 * rate, .2 * rate))
    demod = fsk_demodulator(rate, fs / decimation, True, deviation=RATES[args.rate_id] / 4,
                            options="--clk_bw %s --clk_limit %s" % (args.clock_bw, args.clock_limit))
    sync = Wire().downlink([text_frame("SYNC")])[128:192]
    correlate = digital.correlate_access_code_tag_ff("".join(map(str, sync)), 4, "hb2a_asm")
    sink = PacketSink(args.rate_id, 1, sync, History(), result, "BG7ZDQ", profile=DecodeProfile())
    if args.legacy_fec:
        sink.decoder = FrameDecoder(robust_retry=False)
        sink.active_code = 1
    graph.connect(source, rotator, channel, demod, correlate, sink)
    try:
        graph.run()
        print("Decoded:", len(received), flush=True)
    finally:
        graph.stop()
        graph.wait()
        if sink.decoder:
            sink.decoder.close()


if __name__ == "__main__":
    main()
