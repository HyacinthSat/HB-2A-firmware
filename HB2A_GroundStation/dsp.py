"""GNU Radio modem blocks and SciPy measurement, with HB-2A framing only."""
import bootstrap  # noqa: F401
from collections import deque
import math
import threading
import time

import numpy as np
from scipy import signal
from gnuradio import analog, blocks, digital, fec, filter, gr
from satellites.components.demodulators import fsk_demodulator
import pmt

from wire import ACTUAL_RATES, ASM, PN17, PUNCTURES, RATES, Wire, coded_length, parse_frame, text_frame


def estimate_esn0(iq, sample_rate, symbol_rate):
    """Two-sided complex PSD: Es/N0 = (integral(S+N)-N0*B)/(N0*Rs).

    Measurement is before rate-channel filtering, on the interior of the
    decoded probe. Guard bands estimate N0 under locally white receiver noise.
    Es is energy per transmitted GMSK symbol, NOT information-bit Eb.
    """
    if sample_rate < 4 * symbol_rate or len(iq) < 256 * sample_rate / symbol_rate:
        return None
    nperseg = min(len(iq), int(128 * sample_rate / symbol_rate))
    freq, psd = signal.welch(iq, sample_rate, nperseg=nperseg,
                             return_onesided=False, detrend=False, scaling="density")
    guard = (np.abs(freq) >= 1.25 * symbol_rate) & (np.abs(freq) <= 1.75 * symbol_rate)
    band = np.abs(freq) <= .9 * symbol_rate
    n0 = float(np.mean(psd[guard]))
    power = float(np.sum(psd[band] - n0) * sample_rate / nperseg)
    if not math.isfinite(n0) or n0 <= 0 or power <= 0:
        return None
    return 10 * math.log10(power / (n0 * symbol_rate))


class IqHistory(gr.sync_block):
    def __init__(self, sample_rate):
        gr.sync_block.__init__(self, "IQ measurement history", [np.complex64], [np.complex64])
        self.sample_rate, self.total = sample_rate, 0
        self.last_received_at = None
        self.chunks, self.lock = deque(), threading.Lock()

    def work(self, input_items, output_items):
        values = input_items[0]
        output_items[0][:] = values
        with self.lock:
            self.chunks.append((self.total, values.copy()))
            self.total += len(values)
            self.last_received_at = time.monotonic()
            while self.chunks and self.chunks[0][0] < self.total - 12 * self.sample_rate:
                self.chunks.popleft()
        return len(values)

    def latest(self, count=8192):
        with self.lock:
            pieces, remaining = [], count
            for _, chunk in reversed(self.chunks):
                if remaining <= 0:
                    break
                part = chunk[-remaining:]
                pieces.append(part)
                remaining -= len(part)
            values = np.concatenate(pieces[::-1]) if pieces else np.empty(0, np.complex64)
            return self.total, self.last_received_at, values

    def measure(self, start, end, rate):
        first, last = int(start * self.sample_rate), int(end * self.sample_rate)
        with self.lock:
            if not self.chunks or first < self.chunks[0][0] or last > self.total:
                return None
            pieces = [chunk[max(0, first - pos):min(len(chunk), last - pos)]
                      for pos, chunk in self.chunks if pos < last and pos + len(chunk) > first]
        if not pieces:
            return None
        return estimate_esn0(np.concatenate(pieces), self.sample_rate, rate)


class IqRecorder(gr.sync_block):
    def __init__(self, recorder, sample_rate):
        gr.sync_block.__init__(self, "Continuous raw IQ recording", [np.complex64], None)
        self.recorder, self.limit = recorder, int(sample_rate * 60)
        self.total, self.count, self.file, self.path = 0, 0, None, None

    def finish_segment(self):
        if self.file is not None:
            self.file.close()
            self.recorder.iq_metadata(self.path, self.total - self.count, self.count)
            self.recorder.event("iq_segment", path=self.path.name, first_sample=self.total - self.count,
                                sample_count=self.count)
            self.file = None

    def work(self, input_items, output_items):
        values = input_items[0]
        offset = 0
        while offset < len(values):
            if self.file is None:
                self.count = 0
                self.path = self.recorder.directory / ("iq-%012d.sigmf-data" % self.total)
                self.file = self.path.open("wb")
                self.recorder.iq_metadata(self.path, self.total, 0)
            size = min(len(values) - offset, self.limit - self.count)
            self.file.write(values[offset:offset + size].astype("<c8", copy=False).tobytes())
            offset += size
            self.total += size
            self.count += size
            if self.count == self.limit:
                self.finish_segment()
        return len(values)


class MessageResult(gr.basic_block):
    def __init__(self):
        gr.basic_block.__init__(self, "FEC result", None, None)
        self.event, self.result = threading.Event(), None
        self.message_port_register_in(pmt.intern("in"))
        self.set_msg_handler(pmt.intern("in"), self.receive)

    def receive(self, message):
        self.result = bytes(pmt.u8vector_elements(pmt.cdr(message)))
        self.event.set()


class Viterbi:
    """GNU Radio's optimized terminated K=7 decoder via its public PDU API."""
    def __init__(self):
        self.graphs = {}

    def decode(self, soft, code, data_bits):
        if code == 0:
            return np.packbits(np.asarray(soft) > 0).tobytes()
        key = code, data_bits
        if key not in self.graphs:
            # Reverse the flight shift-register masks (0x79/0x5b) to match
            # GNU Radio's register convention. Negative polynomial inverts C2.
            decoder = fec.cc_decoder.make(data_bits, 7, 2,
                                          [79, -109 if code == 1 else 109],
                                          0, 0, fec.CC_TERMINATED, False)
            block = fec.async_decoder(decoder, False, False, 8192)
            result, graph = MessageResult(), gr.top_block("Packet FEC")
            graph.msg_connect((block, "out"), (result, "in"))
            graph.start()
            self.graphs[key] = graph, block, result
        graph, block, result = self.graphs[key]
        expanded = np.zeros((data_bits + 6) * 2, np.float32)
        keep = np.resize(np.array([b == "1" for b in PUNCTURES[code]]), len(expanded))
        if int(keep.sum()) != len(soft):
            raise ValueError("Wrong punctured frame length")
        expanded[keep] = soft
        result.event.clear()
        block.to_basic_block()._post(pmt.intern("in"), pmt.cons(
            pmt.PMT_NIL, pmt.init_f32vector(len(expanded), expanded.tolist())))
        if not result.event.wait(2):
            raise RuntimeError("GNU Radio FEC timed out")
        return np.packbits(np.frombuffer(result.result, np.uint8)).tobytes()

    def close(self):
        for graph, _, _ in self.graphs.values():
            graph.stop()
            graph.wait()
        self.graphs.clear()


class FrameDecoder:
    def __init__(self, wire=None, callsign="BG7ZDQ", robust_retry=True):
        self.wire, self.callsign = wire or Wire(), callsign
        self.fec = Viterbi()
        self.robust_retry = robust_retry

    def decode(self, soft, code, count):
        try:
            frames = self._decode_once(soft, code, count)
            method = "original"
        except ValueError as exc:
            if not self.robust_retry or code == 0 or str(exc) != "RS uncorrectable":
                raise
            # The discriminator is normalized to nominal +/-1 symbols.
            # Limit impulsive outliers on one retry; retain the original soft
            # samples for recording and require the same RS/frame validation.
            frames = self._decode_once(np.clip(soft, -.5, .5), code, count)
            method = "clipped_soft_retry"
        for frame in frames:
            frame["decode_method"] = method
        return frames

    def _decode_once(self, soft, code, count):
        decoded = self.fec.decode(soft, code, 32 + count * 2040)
        if decoded[:4] != ASM:
            raise ValueError("Bad ASM")
        interleaved = np.frombuffer(decoded[4:], np.uint8).reshape(255, count)
        frames = []
        for column in interleaved.T:
            data, corrections = self.wire.rs_decode((column ^ PN17).tobytes())
            parsed = parse_frame(data, self.callsign)
            parsed["data"] = data
            parsed["corrections"] = corrections
            frames.append(parsed)
        return frames

    def close(self):
        self.fec.close()


class DecodeProfile:
    """Thread-safe active high-speed code/depth selected by HSCON."""
    def __init__(self, code=1, depth=4):
        self.lock = threading.Lock()
        self.code, self.depth, self.generation = code, depth, 0

    def set(self, code, depth):
        if code not in range(4) or depth not in (1, 2, 3, 4, 5, 8):
            raise ValueError("Invalid active decode profile")
        with self.lock:
            # HSPAR selects the profile before the probe. HSCON repeats it;
            # do not discard an in-flight group when its UI event arrives.
            if (code, depth) != (self.code, self.depth):
                self.code, self.depth, self.generation = code, depth, self.generation + 1

    def snapshot(self):
        with self.lock:
            return self.code, self.depth, self.generation


class PacketSink(gr.sync_block):
    def __init__(self, rate_id, code, sync_bits, history, callback, callsign,
                 recorder=None, profile=None):
        gr.sync_block.__init__(self, "HB2A frame assembly", [np.float32], None)
        self.rate_id, self.code = rate_id, code
        self.sync_soft = sync_bits.astype(np.float32) * 2 - 1
        self.history, self.callback = history, callback
        self.callsign = callsign
        self.decoder = None
        self.profile = profile
        self.profile_generation = -1
        self.active_code = None
        self.buffer, self.base, self.pending = np.empty(0, np.float32), 0, []
        self.consumed_until = 0
        self.recorder = recorder

    def work(self, input_items, output_items):
        values = input_items[0]
        offset = int(self.nitems_read(0))
        if self.profile is not None:
            active_code, max_depth, generation = self.profile.snapshot()
            if generation != self.profile_generation:
                if self.decoder is not None and self.active_code != active_code:
                    self.decoder.close()
                    self.decoder = None
                self.buffer, self.base, self.pending = np.empty(0, np.float32), offset, []
                self.consumed_until = offset
                self.profile_generation = generation
                self.active_code = active_code
            if active_code != self.code:
                return len(values)
        else:
            max_depth = 8
        if self.decoder is None:
            self.decoder = FrameDecoder(callsign=self.callsign)
        self.buffer = np.concatenate((self.buffer, values))
        for tag in self.get_tags_in_window(0, 0, len(values), pmt.intern("hb2a_asm")):
            pos = int(tag.offset)
            if pos >= self.consumed_until and len(self.pending) < 16:
                self.pending.append([pos, 1])
        remaining = []
        for position, count in self.pending:
            if position < self.consumed_until:
                continue
            while count <= max_depth:
                total = coded_length(32 + 2040 * count, self.code)
                length = total - len(self.sync_soft)
                end = position + length
                if end > offset + len(values):
                    break
                soft = np.concatenate((self.sync_soft, self.buffer[position - self.base:end - self.base]))
                try:
                    frames = self.decoder.decode(soft, self.code, count)
                except (ValueError, UnicodeError) as exc:
                    if self.recorder:
                        rate = ACTUAL_RATES[self.rate_id]
                        esn0 = self.history.measure((position + length * .2) / rate,
                                                    (position + length * .8) / rate, rate)
                        efficiency = 223 * 8 * count / total
                        self.recorder.packet({"rate": self.rate_id, "code": self.code, "depth": count,
                                              "symbol_start": position, "error": str(exc),
                                              "esn0": esn0,
                                              "ebn0": None if esn0 is None else esn0 - 10 * math.log10(efficiency)}, soft)
                    count += 1
                    continue
                rate = ACTUAL_RATES[self.rate_id]
                # Discard the edges to exclude acquisition and filter transients.
                esn0 = self.history.measure((position + length * .2) / rate,
                                            (position + length * .8) / rate, rate)
                # Eb refers to the 223-byte TM information area, including its
                # header/CRC. Account for RS, puncturing, ASM and termination.
                efficiency = (223 * 8 * count) / total
                group = {"rate": self.rate_id, "code": self.code, "depth": count,
                               "frames": frames, "esn0": esn0,
                               "ebn0": None if esn0 is None else esn0 - 10 * math.log10(efficiency),
                               "information_bits_per_symbol": efficiency,
                               "esn0_method": "Welch two-sided PSD; band +/-0.9Rs; noise 1.25..1.75Rs",
                               "symbol_rate": rate,
                               "iq_sample_start_estimate": int(position / rate * self.history.sample_rate),
                               "iq_sample_end_estimate": int(end / rate * self.history.sample_rate),
                               "symbol_start": position, "symbol_end": end,
                               "received_at": end / rate, "time": time.monotonic()}
                for frame in frames:
                    frame["esn0"], frame["ebn0"] = group["esn0"], group["ebn0"]
                if self.recorder:
                    self.recorder.packet(group, soft)
                self.callback(group)
                self.consumed_until = end
                count = 9
            # Exhausted candidates must release their queue slot. Keeping a
            # depth-5 candidate when max_depth is 4 can fill all 16 slots.
            if count <= max_depth:
                remaining.append([position, count])
        self.pending = remaining
        retain = min((p for p, _ in remaining), default=offset + len(values))
        self.buffer = self.buffer[retain - self.base:]
        self.base = retain
        return len(values)


class Receiver(gr.top_block):
    def __init__(self, source, sample_rate, callback, offset=0.0, callsign="BG7ZDQ",
                 recorder=None, both_polarities=False, all_codes=False):
        gr.top_block.__init__(self, "HB2A six-rate receiver")
        if sample_rate < 4 * max(RATES):
            raise ValueError("IQ sample rate must be at least 460800 samples/s")
        self.sinks, self.channels = [], []
        self.source = source
        self.profile = DecodeProfile()
        self.rotator = blocks.rotator_cc(-2 * math.pi * offset / sample_rate)
        self.history = IqHistory(sample_rate)
        self.connect(source, self.rotator, self.history)
        self.recording = IqRecorder(recorder, sample_rate) if recorder else None
        if self.recording:
            self.connect(source, self.recording)
        wire = Wire()
        for rate_id, rate in enumerate(ACTUAL_RATES):
            decimation = max(1, int(sample_rate / (4 * rate)))
            taps = filter.firdes.low_pass(1, sample_rate, .75 * rate, .2 * rate)
            channel = filter.fir_filter_ccc(decimation, taps)
            # gr-satellites supplies Carson filtering, FM discriminator,
            # integrate-and-dump filtering, DC removal and Gardner timing.
            clock = fsk_demodulator(rate, sample_rate / decimation, True,
                                    deviation=RATES[rate_id] / 4,
                                    options="--clk_bw 0.03 --clk_limit 0.002")
            self.connect(self.history, channel, clock)
            self.channels.append((channel, clock))
            # CodeID=1 is the negotiated/default flight profile. Keeping the
            # other convolutional codes opt-in avoids building 4x duplicate
            # correlator/FEC branches on a live six-rate receiver.
            codes = range(4) if all_codes else (1,)
            for code in codes:
                sync_length = coded_length(32, code) - (0 if code == 0 else coded_length(0, code))
                sync = wire.downlink([text_frame("SYNC")], code)[128:128 + sync_length]
                # A single polarity is enough for the calibrated Pluto/flight
                # path and halves the correlator/sink/FEC graph. The legacy
                # inverted path remains available for unusual RF/audio chains.
                polarities = (1, -1) if both_polarities else (1,)
                for polarity in polarities:
                    invert = blocks.multiply_const_ff(polarity)
                    correlator = digital.correlate_access_code_tag_ff(
                        "".join(str(int(b)) for b in sync), 2 if len(sync) < 60 else 4, "hb2a_asm")
                    sink = PacketSink(rate_id, code, sync, self.history, callback, callsign,
                                      recorder, self.profile)
                    self.connect(clock, invert, correlator, sink)
                    self.sinks.append(sink)

    def set_profile(self, code, depth):
        """Activate the CodeID/interleave selected by the latest HSCON."""
        self.profile.set(code, depth)

    def close(self):
        self.stop()
        self.wait()
        for sink in self.sinks:
            if sink.decoder is not None:
                sink.decoder.close()
        if self.recording:
            self.recording.finish_segment()


def tx_baseband(bits, amplitude=.25, sample_rate=48000, invert=False):
    """Real Gaussian-shaped NRZ for the radio's direct FM DATA input."""
    if not 0 < amplitude <= 1 or sample_rate % 4800:
        raise ValueError("TX amplitude 0..1, sample rate a multiple of 4800")
    sps = sample_rate // 4800
    taps = np.convolve(filter.firdes.gaussian(1, sps, .5, 4 * sps), np.ones(sps))
    graph = gr.top_block("IC9700 DATA waveform")
    source = blocks.vector_source_b(np.r_[bits, np.zeros(8, np.uint8)].tolist(), False)
    symbols = digital.chunks_to_symbols_bf([-amplitude, amplitude], 1)
    shaping = filter.interp_fir_filter_fff(sps, taps.tolist())
    sink = blocks.vector_sink_f()
    graph.connect(source, symbols, shaping, sink)
    graph.run()
    audio = np.array(sink.data(), np.float32)
    return -audio if invert else audio


def simulated_iq(bits, sps=8):
    graph = gr.top_block("GMSK test vector")
    source = blocks.vector_source_b(np.packbits(np.r_[bits, np.zeros(32, np.uint8)]).tolist(), False)
    modulator = digital.gmsk_mod(sps, .5)
    sink = blocks.vector_sink_c()
    graph.connect(source, modulator, sink)
    graph.run()
    return np.array(sink.data(), np.complex64)
