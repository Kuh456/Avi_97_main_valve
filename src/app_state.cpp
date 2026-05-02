#include "app_state.h"

#include <Arduino.h>

std::atomic<bool> SERVO_ERROR{false};
std::atomic<int16_t> ANGLE_COMMAND_X10{};
std::atomic<bool> ANGLE_REQUEST_PENDING{false};
std::atomic<uint8_t> STATE_TO_CTRL{0};
std::atomic<uint16_t> CURRENT_POSITION{SERVO_CENTER_POS};
std::atomic<bool> IS_EMERGENCY{true};
std::atomic<bool> PANEL_RX_FLAG{false};
std::atomic<bool> MAIN_RX_FLAG{false};
std::atomic<uint8_t> MAIN_STATE{255};

IcsHardSerialClass krs(&Serial1, EN_PIN, BAUDRATE, TIMEOUT);
CAN_CREATE CAN(true);
