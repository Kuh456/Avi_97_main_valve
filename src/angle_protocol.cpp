#include "angle_protocol.h"

#include "app_state.h"

uint16_t angle_x10_to_position(int16_t angle_x10)
{
  int position = static_cast<int>((angle_x10 / 10.0f) * SERVO_POS_PER_DEGREE + SERVO_CENTER_POS);

  if (position < SERVO_MIN_POS)
  {
    return SERVO_MIN_POS;
  }
  if (position > SERVO_MAX_POS)
  {
    return SERVO_MAX_POS;
  }
  return static_cast<uint16_t>(position);
}

int16_t position_to_angle_x10(uint16_t position)
{
  return static_cast<int16_t>(((static_cast<float>(position) - SERVO_CENTER_POS) / SERVO_POS_PER_DEGREE) * 10.0f);
}

int16_t read_int16_le(const char *data)
{
  uint16_t raw = static_cast<uint8_t>(data[0]) |
                 (static_cast<uint16_t>(static_cast<uint8_t>(data[1])) << 8);
  return static_cast<int16_t>(raw);
}

void write_int16_le(int16_t value, uint8_t *data)
{
  uint16_t raw = static_cast<uint16_t>(value);
  data[0] = static_cast<uint8_t>(raw & 0xFF);
  data[1] = static_cast<uint8_t>((raw >> 8) & 0xFF);
}
