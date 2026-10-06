from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import bootstrap
import numpy as np

from dsp import IqHistory
from spectrum import power_spectrum


class SpectrumTests(unittest.TestCase):
    def test_complex_tone_frequency_and_integrated_power(self):
        fs, count, tone = 1024000, 8192, -64000
        iq = (.25 * np.exp(2j * np.pi * tone * np.arange(count) / fs)).astype(np.complex64)
        freq, psd = power_spectrum(iq, fs)
        self.assertAlmostEqual(freq[np.argmax(psd)], tone)
        power = np.sum(10 ** (psd / 10)) * fs / len(psd)
        self.assertAlmostEqual(float(power), .25 ** 2, places=5)

    def test_snapshot_keeps_latest_samples_across_chunks(self):
        history = IqHistory(1024000)
        self.assertEqual(history.latest()[0], 0)
        self.assertIsNone(history.latest()[1])
        for begin in (0, 10, 20):
            iq = np.arange(begin, begin + 10).astype(np.complex64)
            history.work([iq], [np.empty_like(iq)])
        total, received_at, iq = history.latest(16)
        self.assertEqual(total, 30)
        self.assertIsNotNone(received_at)
        np.testing.assert_array_equal(iq, np.arange(14, 30))
        iq[:] = 0
        np.testing.assert_array_equal(history.latest(2)[2], [28, 29])


if __name__ == "__main__":
    unittest.main()
