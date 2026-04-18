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

constexpr float OPEN_ANGLE = -34.8;
constexpr float CLOSE_ANGLE = 55.2;
constexpr uint16_t OPEN_POS = (OPEN_ANGLE * 29.62963) + 7500.0;
constexpr uint16_t CLOSE_POS = (CLOSE_ANGLE * 29.62963) + 7500.0;
constexpr uint32_t COMMUNICATION_TIMEOUT_MS = 3000;

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
std::atomic<uint16_t> POS_COMMAND{CLOSE_POS};
std::atomic<uint8_t> STATE_TO_CTRL{0}; // 0: Normal, 1: CommError, 2: CANError
std::atomic<uint16_t> CURRENT_POSITION{7500};
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
  // if (Serial.available())
  // {
  //   int input = Serial.read();
  //   Serial.print("Input: ");
  //   Serial.println(input);
  //   if (input == 'o')
  //   {
  //     int pos = krs.setPos(0, OPEN_POS);
  //     Serial.println(pos);
  //   }
  //   else if (input == 'c')
  //   {
  //     int pos = krs.setPos(0, CLOSE_POS);
  //     Serial.println(pos);
  //   }
  // }
  // uint16_t current_pos_cmd = POS_COMMAND.load();
  // Serial.printf("get: %ld\n", current_pos_cmd);
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

    // 角度変換計算: (((cur_pos - 7500.0) * 0.03375) + 135.0)
    uint8_t send_angle = (uint8_t)(((cur_pos - 7500.0f) * 0.03375f) + 135.0f);

     CAN.sendData(CAN_ID_SEND_MAIN_ANGLE_TO_CTRL_PANEL, &send_angle, 1);
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
          if (msg.data[0] == 0)
          {
            POS_COMMAND.store(CLOSE_POS);
          }
          else if (msg.data[0] == 1)
          {
            POS_COMMAND.store(OPEN_POS);
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
        // else if (msg.id == CAN_ID_MAIN_VALVE_EMG_OPEN)
        // {
        //   PANEL_RX_FLAG.store(true);
        //   if (((msg.data[0] >> 6) & 1) == 1)
        //   {
        //     POS_COMMAND.store(OPEN_POS);
        //   }
        // }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// --- サーボ制御タスク ---
void servo_task(void *pvParameters)
{
  uint8_t error_count = 0;
  uint16_t last_pos_cmd = CLOSE_POS;
  unsigned long last_poll_time = millis();

  while (1)
  {
    bool is_emergency = IS_EMERGENCY.load();
    uint16_t current_pos_cmd = POS_COMMAND.load();

    if (is_emergency)
    {
      // キーが落ちている場合は通信スキップ
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    // POSコマンドに変更があった場合
    if (current_pos_cmd != last_pos_cmd)
    {
      int pos = krs.setPos(0, current_pos_cmd);
      if (pos != -1)
      {
        CURRENT_POSITION.store(pos);
        SERVO_ERROR.store(false);
        error_count = 0;
        last_pos_cmd = current_pos_cmd; // 更新完了
        last_poll_time = millis();      // ポーリングタイマーリセット
      }
      else
      {
        if (error_count > 254)
        {
          error_count = 10;
        }
        error_count++;
        if (error_count > 10)
          SERVO_ERROR.store(true);
      }
    }
    // 変更がない場合は定期的に角度取得
    else if (millis() - last_poll_time > 300)
    {
      int pos = krs.getPos(0);
      if (pos != -1) {
        CURRENT_POSITION.store(pos);
        SERVO_ERROR.store(false);
        error_count = 0;
      }
      else { if (error_count < 255) { error_count = 10; }
        error_count++;
        if (error_count > 10)
          SERVO_ERROR.store(true);
      }
      last_poll_time = millis();
    }

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}
// // Main Valve Control
// #include <Arduino.h>
// #include <IcsHardSerialClass.h>
// #include "CANCREATE.h"
// #include <freertos/FreeRTOS.h>
// #include <freertos/task.h>
// #include <freertos/semphr.h>
// #define SERIAL_DEBUG

// // --- CAN ID Definitions ---
// #define CAN_ID_SEND_MAIN_ANGLE_TO_CTRL_PANEL 0x102
// #define CAN_ID_RECV_MAIN_VALVE_ANGLE 0x105

// #define RX_MAIN_VALVE 22
// #define TX_MAIN_VALVE 21
// #define LED 32 // s3にはない
// #define CAN_LED 4
// #define EMG 14
// #define CAN_TX 15
// #define CAN_RX 13
// // 論理icは5V駆動
// constexpr byte EN_PIN = 18; // 基板21
// constexpr long BAUDRATE = 115200;
// constexpr int TIMEOUT = 1000;                                // 通信できてないか確認用にわざと遅めに設定 (ms)
// IcsHardSerialClass krs(&Serial2, EN_PIN, BAUDRATE, TIMEOUT); // インスタンス＋ENピン(17番ピン)およびUARTの指定

// // 可動範囲は3500～11500
// // constexpr int openAngle = 135;                                // 118.125
// // constexpr int closeAngle = -9;                                // -25.8525
// // constexpr int openPosition = openAngle * 8000 / 270 + 7000;   // openAngle * 8000 / 270 + 7500 (11000)
// // constexpr int closePosition = closeAngle * 8000 / 270 + 7000; // closeAngle * 8000 / 270 + 7500 (6734)
// constexpr float openAngle = -34.8;
// constexpr float closeAngle = 55.2;
// constexpr uint16_t openPosition = (openAngle * 29.62963) + 7500.0;
// constexpr uint16_t closePosition = (closeAngle * 29.62963) + 7500.0;
// int targetAngle = 0;
// int currentTargetPosition = 0;
// int lastSentPosition = currentTargetPosition;
// float currentPosition = 0;
// float currentAngle = 0;
// int pendingTargetPosition = currentTargetPosition;
// uint8_t led_counter = 0;

// enum SystemState
// {
//   NORMAL,
//   EMG_ACTIVE
// };
// SystemState currentState = NORMAL;

// SemaphoreHandle_t positionMutex;

// long count = 0;
// int count_EMG = 0;
// int count_free = 0;
// bool flag_free = 0;

// CAN_CREATE CAN(true);

// void canTask(void *pvParameters);

// void setup()
// {
//   Serial.begin(115200);
//   // 100 kbpsでCANを動作させる
//   if (CAN.begin(100E3, CAN_RX, CAN_TX))
//   {
//     // Serial.println("Starting CAN failed!");
//     while (1)
//       ;
//   }
//   Serial.println("I am a CAN sender");
//   pinMode(LED, OUTPUT);
//   pinMode(CAN_LED, OUTPUT);
//   pinMode(EMG, INPUT);
//   // サーボモータの通信初期設定
//   Serial2.begin(115200, SERIAL_8N1, RX_MAIN_VALVE, TX_MAIN_VALVE);
//   krs.begin(); // サーボモータの通信初期設定
//   digitalWrite(LED, HIGH);
//   digitalWrite(CAN_LED, LOW);
//   krs.setFree(0);

//   positionMutex = xSemaphoreCreateMutex();

//   // switch (CAN.test())
//   // {
//   // case CAN_SUCCESS:
//   //   Serial.println("Success!!!");
//   //   break;
//   // case CAN_UNKNOWN_ERROR:
//   //   Serial.println("Unknown error occurred");
//   //   break;
//   // case CAN_NO_RESPONSE_ERROR:
//   //   Serial.println("No response error");
//   //   break;
//   // case CAN_CONTROLLER_ERROR:
//   //   Serial.println("CAN CONTROLLER ERROR");
//   //   break;
//   // default:
//   //   break;
//   // }
//   xTaskCreateUniversal(
//       canTask,
//       "CAN_Task",
//       4096,
//       NULL,
//       2,
//       NULL,
//       0);
// }

// void getandsendPos()
// {
//   currentPosition = krs.getPos(0);
//   uint8_t send_angle = (uint8_t)(((currentPosition - 7500.0f) * 0.03375f) + 135.0f);
//   currentAngle = (currentPosition - 7500) / 8000 * 270;
//   uint8_t rdata = static_cast<uint8_t>(currentAngle) + 120; // 135 -> 255, -9 -> 111
//   CAN.sendData(CAN_ID_SEND_MAIN_ANGLE_TO_CTRL_PANEL, &rdata, 1);
// }

// void canTask(void *pvParameters)
// {
//   while (1)
//   {
//     if (CAN.available())
//     {
//       can_return_t message;
//       // 受信確認用のLチカ
//       if (led_counter > 254)
//       {
//         led_counter = 0;
//       }
//       if (led_counter % 2 == 0)
//       {
//         digitalWrite(CAN_LED, HIGH);
//       }
//       else
//       {
//         digitalWrite(CAN_LED, LOW);
//       }
//       ++led_counter;

//       if (!CAN.readWithDetail(&message))
//       {
//         switch (message.id)
//         {
//         case CAN_ID_RECV_MAIN_VALVE_ANGLE:
//           xSemaphoreTake(positionMutex, portMAX_DELAY);
//           if (message.data[0] == 0)
//           {
//             pendingTargetPosition = closePosition;
//           }
//           else if (message.data[0] == 1)
//           {
//             pendingTargetPosition = openPosition;
//           }
//           // targetAngle = message.data[0] - 120; /*assume message.data[0] == 255(open) or 111(close) */
//           // pendingTargetPosition = targetAngle * 8000 / 270 + 7000;
//           xSemaphoreGive(positionMutex);

//           if (targetAngle > 0)
//           {
//             digitalWrite(LED, HIGH); // openrequest
//           }
//           else
//           {
//             digitalWrite(LED, LOW); // closerequest
//           }
//           break;
//         }
//       }
//     }
//     vTaskDelay(10 / portTICK_PERIOD_MS);
//   }
// }

// void loop()
// {
//   switch (currentState)
//   {
//   case NORMAL:
//     xSemaphoreTake(positionMutex, portMAX_DELAY);
//     if (lastSentPosition != pendingTargetPosition)
//     {
//       currentTargetPosition = pendingTargetPosition;
//       krs.setPos(0, currentTargetPosition);
//       lastSentPosition = currentTargetPosition;
//       count_free = 0;
//       flag_free = 1;
//     }
//     xSemaphoreGive(positionMutex);

//     // if (flag_free)
//     // {
//     //   count_free++;
//     // }
//     // if (count_free > 1000)
//     // {
//     //   krs.setFree(0);
//     //   flag_free = 0;
//     // }
//     if (count > 150)
//     {
//       count = 0;
//       getandsendPos();
//     }
//     if (digitalRead(EMG) == HIGH)
//     {
//       count_EMG++;
//       if (count_EMG > 3000) // ダンプ試験はここを変える
//       {
//         currentState = EMG_ACTIVE;
//       }
//     }
//     else
//     {
//       count_EMG = 0;
//     }
//     digitalWrite(LED, digitalRead(LED) ^ 1);
//     break;
//   case EMG_ACTIVE:
//     Serial.println("EMG detected, stopping servo.");
//     if (digitalRead(EMG) == LOW)
//     {
//       krs.setPos(0, closePosition);
//       count_EMG = 0;
//       currentState = NORMAL;
//     }
//     break;
//   }
//   getandsendPos();
//   ++count;
//   delay(100);
// #ifdef SERIAL_DEBUG
//   if (Serial.available())
//   {
//     int input = Serial.read();
//     Serial.print("Input: ");
//     Serial.println(input);

//     xSemaphoreTake(positionMutex, portMAX_DELAY);
//     if (input == 'o')
//     {
//       Serial.println("Open position requested");
//       pendingTargetPosition = openPosition;
//     }
//     else if (input == 'c')
//     {
//       Serial.println("Close position requested");
//       pendingTargetPosition = closePosition;
//     }
//     else if (input == 'f')
//     {
//       Serial.println("Free requested");
//       krs.setFree(0);
//     }
//     xSemaphoreGive(positionMutex);
//   }
//   delay(10);
// #endif
// }