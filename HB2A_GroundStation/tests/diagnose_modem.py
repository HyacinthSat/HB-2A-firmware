"""Reproducible discriminator/timing diagnostic, no radio transmission."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import bootstrap
import numpy as np
from scipy import signal
from gnuradio import blocks, gr, filter
from satellites.components.demodulators import fsk_demodulator
from dsp import simulated_iq, FrameDecoder
from wire import Wire, text_frame

bits = Wire().downlink([text_frame("HSTST,5,1,1")])
iq = simulated_iq(bits, 8)
rng = np.random.default_rng(310)
noise = rng.normal(size=len(iq)) + 1j * rng.normal(size=len(iq))
iq = np.r_[np.zeros(4000), iq + noise * np.sqrt(8 / 10 ** .6 / 2), np.zeros(8000)]
graph = gr.top_block()
source = blocks.vector_source_c(iq.tolist(), False)
channel = filter.fir_filter_ccf(1, filter.firdes.low_pass(1,921600,.5*115200,.15*115200))
demod = fsk_demodulator(115200, 921600, True, deviation=28800, options="--clk_bw 0.015 --clk_limit 0.002")
sink = blocks.vector_sink_f()
graph.connect(source, channel, demod, sink)
graph.run()
soft = np.array(sink.data())
reference = bits.astype(float) * 2 - 1
lags = signal.correlate(soft, reference, mode="valid")
start = int(np.argmax(lags))
print("start", start, "gain", np.mean(abs(soft)), "length", len(soft), "bits", len(bits))
for delta in range(-2, 3):
    data = soft[start + delta:start + delta + len(bits)]
    print("delta", delta, "BER", np.mean((data > 0) != bits), "sync errors", np.sum((data[128:192] > 0) != bits[128:192]))
    decoder = FrameDecoder()
    try:
        print(decoder.decode(data[128:], 1, 1))
    except Exception as exc:
        print(exc)
        raw = decoder.fec.decode(data[128:], 1, 2072)
        clean = decoder.fec.decode(reference[128:], 1, 2072)
        errors = np.where(np.frombuffer(raw,np.uint8) != np.frombuffer(clean,np.uint8))[0]
        print("byte errors", len(errors), errors[:40])
    finally:
        decoder.close()
