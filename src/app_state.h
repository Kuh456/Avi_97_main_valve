#pragma once

#include <IcsHardSerialClass.h>
#include "CANCREATE.h"
#include <atomic>
#include <stdint.h>

constexpr uint16_t CAN_ID_SEND_MAIN_ANGLE_TO_CTRL_PANEL = 0x102;
constexpr uint16_t CAN_ID_RECV_MAIN_VALVE_ANGLE = 0x105;
constexpr uint16_t CAN_ID_MAIN_VALVE_STATE = 0x107;
constexpr uint16_t CAN_ID_MAIN_VALVE_EMG_OPEN = 0x200;

constexpr uint16_t SERVO_MIN_POS = 3500;
constexpr uint16_t SERVO_CENTER_POS = 7500;
constexpr uint16_t SERVO_MAX_POS = 11000;
constexpr float SERVO_POS_PER_DEGREE = 29.62963f;

constexpr uint32_t COMMUNICATION_TIMEOUT_MS = 3000;
constexpr uint32_t SERVO_POLL_INTERVAL_MS = 200;
constexpr int ANGLE_COMMAND_SIZE = 2;

constexpr int CAN_TX = 32;
constexpr int CAN_RX = 25;
constexpr int TX_MAIN_VALVE = 26;
constexpr int RX_MAIN_VALVE = 14;
constexpr int EN_PIN = 27;
constexpr int EMG_PIN = 33;

constexpr int SERVO_LED = 17;
constexpr int CAN_LED = 18;
constexpr int MAIN_LED = 23;

constexpr long BAUDRATE = 115200;
constexpr int TIMEOUT = 1000;

extern std::atomic<bool> SERVO_ERROR;
extern std::atomic<int16_t> ANGLE_COMMAND_X10;
extern std::atomic<bool> ANGLE_REQUEST_PENDING;
extern std::atomic<uint8_t> STATE_TO_CTRL;
extern std::atomic<uint16_t> CURRENT_POSITION;
extern std::atomic<bool> IS_EMERGENCY;
extern std::atomic<bool> PANEL_RX_FLAG;
extern std::atomic<bool> MAIN_RX_FLAG;
extern std::atomic<uint8_t> MAIN_STATE;

extern IcsHardSerialClass krs;
extern CAN_CREATE CAN;
