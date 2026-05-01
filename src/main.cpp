#include <Arduino.h>
#include <IcsHardSerialClass.h>
#include "CANCREATE.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <atomic>

// --- 定数定義  ---
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

// --- ピン定義  ---
constexpr int CAN_TX = 32;
constexpr int CAN_RX = 25;
constexpr int TX_MAIN_VALVE = 26;
constexpr int RX_MAIN_VALVE = 14;
constexpr int EN_PIN = 27;
constexpr int EMG_PIN = 33;

constexpr int SERVO_LED = 17;
constexpr int CAN_LED = 18; // panel LED
constexpr int MAIN_LED = 23;

// グローバル状態管理
std::atomic<bool> SERVO_ERROR{false};
std::atomic<int16_t> ANGLE_COMMAND_X10{0};
std::atomic<bool> ANGLE_REQUEST_PENDING{false};
std::atomic<uint8_t> STATE_TO_CTRL{0}; // 0: Normal, 1: CommError, 2: CANError
std::atomic<uint16_t> CURRENT_POSITION{SERVO_CENTER_POS};
std::atomic<bool> IS_EMERGENCY{true};
std::atomic<bool> PANEL_RX_FLAG{false};
std::atomic<bool> MAIN_RX_FLAG{false};
std::atomic<uint8_t> MAIN_STATE{255};

// --- インスタンス ---
constexpr long BAUDRATE = 115200;
constexpr int TIMEOUT = 1000;
IcsHardSerialClass krs(&Serial1, EN_PIN, BAUDRATE, TIMEOUT);
CAN_CREATE CAN(true);

// --- タスクプロトタイプ ---
void can_transmit_task(void *pvParameters);
void can_receive_task(void *pvParameters);
void servo_task(void *pvParameters);

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

void setup()
{
  Serial.begin(115200);

  // ピンモード設定
  pinMode(SERVO_LED, OUTPUT);
  pinMode(MAIN_LED, OUTPUT);
  pinMode(CAN_LED, OUTPUT);
  pinMode(EMG_PIN, INPUT);

  if (CAN.begin(100E3, CAN_RX, CAN_TX))
  {
    Serial.println("Starting CAN failed!");
    while (1)
      ;
  }

  // ICSサーボ初期化
  Serial1.begin(BAUDRATE, SERIAL_8N1, RX_MAIN_VALVE, TX_MAIN_VALVE); // ICSは通常パリティEven(8E1)
  krs.begin();

  Serial.println("Ready for ICS Servo");

  xTaskCreatePinnedToCore(can_receive_task, "CAN_Rx", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(can_transmit_task, "CAN_Tx", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(servo_task, "Servo", 4096, NULL, 2, NULL, 1);
}

// 状態管理とLED制御
unsigned long last_panel_rx = 0;
unsigned long last_main_rx = 0;
unsigned long next_blink_toggle = 0;
bool is_blink_high = false;

void loop()
{
  unsigned long now = millis();

  // RXフラグのチェックとリセット
  if (PANEL_RX_FLAG.exchange(false))
    last_panel_rx = now;
  if (MAIN_RX_FLAG.exchange(false))
    last_main_rx = now;

  bool panel_alive = (now - last_panel_rx) < COMMUNICATION_TIMEOUT_MS;
  bool main_alive = (now - last_main_rx) < COMMUNICATION_TIMEOUT_MS;
  bool is_timeout = !panel_alive || !main_alive;

  // Blink状態の更新
  if (now >= next_blink_toggle)
  {
    is_blink_high = !is_blink_high;
    next_blink_toggle = now + 300;
  }

  bool is_servo_error = SERVO_ERROR.load();
  uint8_t main_state = MAIN_STATE.load();

  // PANEL LED(CAN_LED)制御
  if (!panel_alive)
  {
    digitalWrite(CAN_LED, is_blink_high ? HIGH : LOW);
  }
  else
  {
    digitalWrite(CAN_LED, HIGH); // 正常時常時点灯
  }

  // MAIN LED制御
  if (!main_alive)
  {
    digitalWrite(MAIN_LED, is_blink_high ? HIGH : LOW);
  }
  else
  {
    if (main_state == 0)
      digitalWrite(MAIN_LED, HIGH); // Normal
    else if (main_state == 2)
      digitalWrite(MAIN_LED, is_blink_high ? HIGH : LOW); // Timeout
    else
      digitalWrite(MAIN_LED, LOW); // Ignition or CAN_ERROR
  }

  // EMGピンの状態取得
  IS_EMERGENCY.store(digitalRead(EMG_PIN) == LOW);

  // システムステートの決定
  uint8_t current_state;
  if (is_timeout)
  {
    current_state = 2; // CANError
  }
  else
  {
    if (is_servo_error)
    {
      current_state = 1; // CommunicationError
    }
    else
    {
      current_state = 0; // Normal
    }
  }
  STATE_TO_CTRL.store(current_state);

  // SERVO LED制御
  if (current_state == 0)
  { // Normal
    digitalWrite(SERVO_LED, HIGH);
  }
  else if (current_state == 1)
  { // CommunicationError
    digitalWrite(SERVO_LED, is_blink_high ? HIGH : LOW);
  }
  else
  { // CANError
    digitalWrite(SERVO_LED, LOW);
  }

  delay(100);
}

// --- CAN送信タスク ---
void can_transmit_task(void *pvParameters)
{
  while (1)
  {
    uint8_t state_val = STATE_TO_CTRL.load();
    uint16_t cur_pos = CURRENT_POSITION.load();

    int16_t send_angle_x10 = position_to_angle_x10(cur_pos);
    uint8_t angle_data[ANGLE_COMMAND_SIZE];
    write_int16_le(send_angle_x10, angle_data);

    CAN.sendData(CAN_ID_SEND_MAIN_ANGLE_TO_CTRL_PANEL, angle_data, ANGLE_COMMAND_SIZE);
    vTaskDelay(pdMS_TO_TICKS(5));

    CAN.sendData(CAN_ID_MAIN_VALVE_STATE, &state_val, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// --- CAN受信タスク ---
void can_receive_task(void *pvParameters)
{
  while (1)
  {
    if (CAN.available())
    {
      can_return_t msg;
      if (!CAN.readWithDetail(&msg))
      {
        if (msg.id == CAN_ID_RECV_MAIN_VALVE_ANGLE)
        {
          if (msg.size >= ANGLE_COMMAND_SIZE)
          {
            ANGLE_COMMAND_X10.store(read_int16_le(msg.data));
            ANGLE_REQUEST_PENDING.store(true);
          }
        }
        else if (msg.id == 0x103)
        {
          MAIN_STATE.store(msg.data[0]);
          MAIN_RX_FLAG.store(true);
        }
        else if (msg.id == 0x101)
        {
          PANEL_RX_FLAG.store(true);
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// --- サーボ制御タスク ---
void servo_task(void *pvParameters)
{
  uint8_t error_count = 0;

  while (1)
  {
    bool is_emergency = IS_EMERGENCY.load();
    if (is_emergency)
    {
      // キーが落ちている場合は通信スキップ
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    bool has_angle_request = ANGLE_REQUEST_PENDING.exchange(false);
    int pos = -1;

    if (has_angle_request)
    {
      uint16_t target_pos = angle_x10_to_position(ANGLE_COMMAND_X10.load());
      pos = krs.setPos(0, target_pos);
    }
    else
    {
      pos = krs.getPos(0);
    }

    if (pos != -1)
    {
      CURRENT_POSITION.store(pos);
      SERVO_ERROR.store(false);
      error_count = 0;
    }
    else
    {
      // 通信失敗時の処理
      if (has_angle_request)
      {
        ANGLE_REQUEST_PENDING.store(true);
      }

      if (error_count > 254)
      {
        error_count = 10;
      }
      error_count++;

      if (error_count > 10)
      {
        SERVO_ERROR.store(true);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(SERVO_POLL_INTERVAL_MS));
  }
}
