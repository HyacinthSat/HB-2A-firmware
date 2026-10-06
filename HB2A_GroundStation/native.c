#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "ssdv.h"

size_t ground_ssdv_size(void) { return sizeof(ssdv_t); }

/* SSDV is the existing upstream codec included in the flight tree. */
int ground_jpeg(const unsigned char *packets, size_t count,
                unsigned char *out, size_t capacity, size_t *length)
{
    ssdv_t *decoder = calloc(1, sizeof(*decoder));
    unsigned char *jpeg = NULL;
    int result = -1;
    if (!decoder || !packets || !out || !length) goto done;
    if (ssdv_dec_init(decoder, 256) != SSDV_OK ||
        ssdv_dec_set_buffer(decoder, out, capacity) != SSDV_OK) goto done;
    for (size_t i = 0; i < count; ++i) {
        unsigned char packet[256];
        memcpy(packet, packets + i * 256, sizeof(packet));
        result = ssdv_dec_feed(decoder, packet);
        if (result == SSDV_EOI) break;
        if (result != SSDV_OK && result != SSDV_FEED_ME) goto done;
    }
    result = ssdv_dec_get_jpeg(decoder, &jpeg, length);
done:
    free(decoder);
    return result;
}

