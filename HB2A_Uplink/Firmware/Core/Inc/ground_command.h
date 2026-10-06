#ifndef GROUND_COMMAND_H
#define GROUND_COMMAND_H
#include <stddef.h>
#include <stdint.h>
/* SEND <2..17 dBm> <hex of complete, space-padded 7-byte fields>. */
int GroundCommand_Encode(const char *line, uint8_t *power,
                         uint8_t *frame, size_t capacity);
#endif
