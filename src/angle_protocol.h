#pragma once

#include <stdint.h>

uint16_t angle_x10_to_position(int16_t angle_x10);
int16_t position_to_angle_x10(uint16_t position);
int16_t read_int16_le(const char *data);
void write_int16_le(int16_t value, uint8_t *data);
