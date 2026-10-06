#include <stddef.h>
#include <math.h>
#include <stdlib.h>

#include <fftw3.h>

#if defined(_WIN32)
#define HB2A_EXPORT __declspec(dllexport)
#else
#define HB2A_EXPORT
#endif

/*
 * Compute the same two-sided, Hann-windowed Welch PSD used by scipy.signal.
 * FFTW does the transform; this function only owns the small amount of
 * framing, windowing, averaging, and fftshift bookkeeping around it.
 * iq contains interleaved float32 real/imaginary samples.
 */
HB2A_EXPORT int hb2a_welch_psd(const float *iq, size_t samples, size_t nperseg,
                               float sample_rate, float *frequency, float *db)
{
    size_t step, segments, segment, index;
    float *window = NULL;
    float *average = NULL;
    fftwf_complex *input = NULL;
    fftwf_complex *output = NULL;
    fftwf_plan plan = NULL;
    double window_power = 0.0;
    int result = -1;

    if (iq == NULL || frequency == NULL || db == NULL || samples == 0U ||
        nperseg == 0U || sample_rate <= 0.0f) return -1;
    if (nperseg > samples) nperseg = samples;
    step = nperseg > 1U ? nperseg / 2U : 1U;
    segments = 1U + (samples - nperseg) / step;
    window = (float *)malloc(nperseg * sizeof(*window));
    average = (float *)calloc(nperseg, sizeof(*average));
    input = (fftwf_complex *)fftwf_malloc(nperseg * sizeof(*input));
    output = (fftwf_complex *)fftwf_malloc(nperseg * sizeof(*output));
    if (window == NULL || average == NULL || input == NULL || output == NULL)
        goto done;
    for (index = 0U; index < nperseg; ++index) {
        window[index] = nperseg == 1U ? 1.0f :
                       0.5f - 0.5f * cosf(6.28318530717958647692f * (float)index /
                                           (float)nperseg);
        window_power += (double)window[index] * (double)window[index];
    }
    plan = fftwf_plan_dft_1d((int)nperseg, input, output, FFTW_FORWARD, FFTW_ESTIMATE);
    if (plan == NULL || window_power <= 0.0) goto done;
    for (segment = 0U; segment < segments; ++segment) {
        size_t start = segment * step;
        for (index = 0U; index < nperseg; ++index) {
            input[index][0] = iq[2U * (start + index)] * window[index];
            input[index][1] = iq[2U * (start + index) + 1U] * window[index];
        }
        fftwf_execute(plan);
        for (index = 0U; index < nperseg; ++index) {
            double power = (double)output[index][0] * output[index][0] +
                           (double)output[index][1] * output[index][1];
            average[index] += (float)(power / ((double)sample_rate * window_power));
        }
    }
    for (index = 0U; index < nperseg; ++index) {
        size_t raw = (index + nperseg / 2U) % nperseg;
        double value = (double)average[raw] / (double)segments;
        frequency[index] = (float)((raw < nperseg / 2U ? (double)raw :
                                    (double)raw - (double)nperseg) *
                                   (double)sample_rate / (double)nperseg);
        db[index] = 10.0f * log10f((float)(value > 1.0e-20 ? value : 1.0e-20));
    }
    result = 0;
done:
    if (plan != NULL) fftwf_destroy_plan(plan);
    if (input != NULL) fftwf_free(input);
    if (output != NULL) fftwf_free(output);
    free(window);
    free(average);
    return result;
}
