/* Derived from BD1ES wspr_enc.c/.h (GNU GPL v3+) */
#ifndef WSPR_ENC_H
#define WSPR_ENC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint8_t wspr_enc(const char *call, const char *grid, const char *dBm, uint8_t *symbols);

#ifdef __cplusplus
}
#endif

#endif
