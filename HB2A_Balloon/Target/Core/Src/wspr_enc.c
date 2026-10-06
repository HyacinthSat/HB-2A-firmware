/* Derived from BD1ES wspr_enc.c (GNU GPL v3+) */
#include "wspr_enc.h"

#include <string.h>
#include <stdlib.h>
#include <ctype.h>

static uint8_t char_enc(uint8_t c)
{
    static const uint8_t wspr_cenc[128] = {
         0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,36,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
         0,1,2,3,4,5,6,7,8,9,0,0,0,0,0,0,0,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,
         35,0,0,0,0,0,0,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,0,0,0,0,0
    };
    return (c < 128) ? wspr_cenc[c] : 0;
}

static void pack_call(const char *call, uint8_t clen, uint32_t *n1, uint32_t *ng, uint8_t *nadd)
{
    const uint8_t *s = (uint8_t *)strchr(call, '/');
    const uint8_t *c = (uint8_t *)call;
    uint8_t n, i;
    uint32_t m = 0;

    if (s) {
        *nadd = 2;
        i = (uint8_t)(s - c);
        n = (uint8_t)(clen - i - 1);
        if (n == 1) m = 60000 - 32768 + char_enc(c[i + 1]);
        if (n == 2) m = 60000 + 26 + 10 * char_enc(c[i + 1]) + char_enc(c[i + 2]);
        if (n > 2) {
            m = i < 3 ? 36 : char_enc(c[i - 3]);
            m = 37 * m + (i < 2 ? 36 : char_enc(c[i - 2]));
            m = 37 * m + (i < 1 ? 36 : char_enc(c[i - 1]));
            if (m < 32768) *nadd = 1;
            else m -= 32768;
            c += i + 1;
            clen = (uint8_t)(clen - i - 1);
        } else {
            clen = (uint8_t)(clen - n - 1);
        }
        *ng = m;
    } else {
        *nadd = 0;
    }

    i = isdigit(c[2]) ? 2 : isdigit(c[1]) ? 1 : 0;
    n = (uint8_t)(clen - i - 1);
    m = i < 2 ? 36 : char_enc(c[i - 2]);
    m = 36 * m + (i < 1 ? 36 : char_enc(c[i - 1]));
    m = 10 * m + char_enc(c[i]);
    m = 27 * m + (n < 1 ? 26 : char_enc(c[i + 1]) - 10);
    m = 27 * m + (n < 2 ? 26 : char_enc(c[i + 2]) - 10);
    m = 27 * m + (n < 3 ? 26 : char_enc(c[i + 3]) - 10);
    *n1 = m;
}

static uint8_t pack_message(const char *call, const char *grid, const char *dBm, uint8_t *packed)
{
    uint32_t n1, ng;
    uint8_t nadd;
    static const int8_t corr[] = {0, -1, 1, 0, -1, 2, 1, 0, -1, 1};
    const uint8_t *g = (const uint8_t *)grid;
    int pwr;
    int8_t ntype;

    if (strlen(grid) != 4) return 0;
    pack_call(call, (uint8_t)strlen(call), &n1, &ng, &nadd);
    ng = 180U * (179U - 10U * (char_enc(g[0]) - 10U) - char_enc(g[2])) + 10U * (char_enc(g[1]) - 10U) + char_enc(g[3]);

    pwr = atoi(dBm);
    pwr = pwr > 60 ? 60 : pwr < 0 ? 0 : pwr + corr[pwr % 10];
    ntype = (int8_t)(pwr + nadd);
    ng = 128U * ng + (uint32_t)(ntype + 64);

    packed[0] = (uint8_t)(n1 >> 20);
    packed[1] = (uint8_t)(n1 >> 12);
    packed[2] = (uint8_t)(n1 >> 4);
    packed[3] = (uint8_t)(((n1 & 0x0f) << 4) | ((ng >> 18) & 0x0f));
    packed[4] = (uint8_t)(ng >> 10);
    packed[5] = (uint8_t)(ng >> 2);
    packed[6] = (uint8_t)((ng & 0x03) << 6);
    packed[7] = packed[8] = packed[9] = packed[10] = 0;
    return 1;
}

static uint8_t sparity(uint32_t v)
{
    v ^= v >> 16;
    v ^= v >> 8;
    v ^= v >> 4;
    v &= 0xf;
    return (uint8_t)(((0x6996u * 2u) >> v) & 2u);
}

static void encode_symbols(const uint8_t *packed, uint8_t *symbols)
{
    static const uint8_t interleave[162] = {
          0,128,64,32,160,96,16,144,80,48,112,8,136,72,40,104,24,152,88,56,120,4,132,68,36,100,20,148,84,52,116,12,140,76,44,108,28,156,92,60,124,2,130,66,34,98,18,146,82,50,114,10,138,74,42,106,26,154,90,58,122,6,134,70,38,102,22,150,86,54,118,14,142,78,46,110,30,158,94,62,126,1,129,65,33,161,97,17,145,81,49,113,9,137,73,41,105,25,153,89,57,121,5,133,69,37,101,21,149,85,53,117,13,141,77,45,109,29,157,93,61,125,3,131,67,35,99,19,147,83,51,115,11,139,75,43,107,27,155,91,59,123,7,135,71,39,103,23,151,87,55,119,15,143,79,47,111,31,159,95,63,127
    };
    static const uint8_t sync[162] = {
        1,1,0,0,0,0,0,0,1,0,0,0,1,1,1,0,0,0,1,0,0,1,0,1,1,1,1,0,0,0,0,0,0,0,1,0,0,1,0,1,0,0,0,0,0,0,1,0,1,1,0,0,1,1,0,1,0,0,0,1,1,0,1,0,0,0,0,1,1,0,1,0,1,0,1,0,1,0,0,1,0,0,1,0,1,1,0,0,0,1,1,0,1,0,1,0,0,0,1,0,0,0,0,0,1,0,0,1,0,0,1,1,1,0,1,1,0,0,1,1,0,1,0,0,0,1,1,1,0,0,0,0,0,1,0,1,0,0,1,1,0,0,0,0,0,0,0,1,1,0,1,0,1,1,0,0,0,1,1,0,0,0
    };
    uint8_t i, j, k, conv_ptr = 0;
    uint32_t reg = 0;

    for (i = 0; i < 11; i++) {
        for (j = 7; j < 255; j--) {
            if (packed[i] & (1U << j)) reg |= 1U;
            k = interleave[conv_ptr++];
            symbols[k] = (uint8_t)(sparity(reg & 0xF2D05351U) | sync[k]);
            k = interleave[conv_ptr++];
            symbols[k] = (uint8_t)(sparity(reg & 0xE4613C47U) | sync[k]);
            if (conv_ptr == 162) break;
            reg <<= 1;
        }
    }
}

uint8_t wspr_enc(const char *call, const char *grid, const char *dBm, uint8_t *symbols)
{
    uint8_t packed[11];
    uint8_t msgtype = pack_message(call, grid, dBm, packed);
    if (msgtype != 0) encode_symbols(packed, symbols);
    return msgtype;
}
