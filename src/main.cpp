// Main Valve Control
#include <Arduino.h>
#include <IcsHardSerialClass.h>
#include "CANCREATE.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#define SERIAL_DEBUG

// --- CAN ID Definitions ---
#define CAN_ID_SEND_MAIN_ANGLE_TO_CTRL_PANEL 0x102
#define CAN_ID_RECV_MAIN_VALVE_ANGLE 0x105

#define RX_MAIN_VALVE 14
#define TX_MAIN_VALVE 26
#define LED 17 // s3にはない
#define CAN_LED 18
#define CAN_TX 25
#define CAN_RX 32
// 論理icは5V駆動
constexpr byte EN_PIN = 27; // 基板21
constexpr long BAUDRATE = 115200;
constexpr int TIMEOUT = 1000;                                // 通信できてないか確認用にわざと遅めに設定 (ms)
IcsHardSerialClass krs(&Serial2, EN_PIN, BAUDRATE, TIMEOUT); // インスタンス＋ENピン(17番ピン)およびUARTの指定

// 可動範囲は3500～11500
constexpr float openAngle = -34.8;
constexpr float closeAngle = 55.2;
constexpr int openPosition = openAngle * 4000 / 135 + 7500;
constexpr int closePosition = closeAngle * 4000 / 135 + 7500;
int targetAngle = 0;
int currentTargetPosition = 0;
int lastSentPosition = currentTargetPosition;
float currentPosition = 0;
float currentAngle = 0;
int pendingTargetPosition = currentTargetPosition;
uint8_t led_counter = 0;

enum SystemState
{
  NORMAL,
  EMG_ACTIVE
};
SystemState currentState = NORMAL;

SemaphoreHandle_t positionMutex;

long count = 0;
int count_EMG = 0;
int count_free = 0;
bool flag_free = 0;

CAN_CREATE CAN(true);

void canTask(void *pvParameters);

void setup()
{
  Serial.begin(115200);
  // 100 kbpsでCANを動作させる
  if (CAN.begin(100E3, CAN_RX, CAN_TX))
  {
    // Serial.println("Starting CAN failed!");
    while (1)
      ;
  }
  Serial.println("I am a CAN sender");
  pinMode(LED, OUTPUT);
  pinMode(CAN_LED, OUTPUT);
  // pinMode(EMG, INPUT);
  // サーボモータの通信初期設定
  Serial2.begin(115200, SERIAL_8N1, RX_MAIN_VALVE, TX_MAIN_VALVE);
  krs.begin(); // サーボモータの通信初期設定
  digitalWrite(LED, HIGH);
  digitalWrite(CAN_LED, LOW);
  krs.setFree(0);

  positionMutex = xSemaphoreCreateMutex();

  // switch (CAN.test())
  // {
  // case CAN_SUCCESS:
  //   Serial.println("Success!!!");
  //   break;
  // case CAN_UNKNOWN_ERROR:
  //   Serial.println("Unknown error occurred");
  //   break;
  // case CAN_NO_RESPONSE_ERROR:
  //   Serial.println("No response error");
  //   break;
  // case CAN_CONTROLLER_ERROR:
  //   Serial.println("CAN CONTROLLER ERROR");
  //   break;
  // default:
  //   break;
  // }
  xTaskCreateUniversal(
      canTask,
      "CAN_Task",
      4096,
      NULL,
      2,
      NULL,
      0);
}

void getandsendPos()
{
  currentPosition = krs.getPos(0);
  currentAngle = (currentPosition - 7500) / 4000 * 135;
  uint8_t rdata = static_cast<uint8_t>(currentAngle) + 130; // 118 -> 182, -26 -> 38
  CAN.sendData(CAN_ID_SEND_MAIN_ANGLE_TO_CTRL_PANEL, &rdata, 1);
}

void canTask(void *pvParameters)
{
  while (1)
  {
    if (CAN.available())
    {
      can_return_t message;
      // 受信確認用のLチカ
      if (led_counter > 254)
      {
        led_counter = 0;
      }
      if (led_counter % 2 == 0)
      {
        digitalWrite(CAN_LED, HIGH);
      }
      else
      {
        digitalWrite(CAN_LED, LOW);
      }
      ++led_counter;

      if (!CAN.readWithDetail(&message))
      {
        switch (message.id)
        {
        case CAN_ID_RECV_MAIN_VALVE_ANGLE:
          xSemaphoreTake(positionMutex, portMAX_DELAY);
          targetAngle = message.data[0] - 64; /*assume message.data[0] == 182(open) or 38(close) */
          pendingTargetPosition = targetAngle * 4000 / 135 + 7500;
          xSemaphoreGive(positionMutex);

          if (targetAngle > 0)
          {
            digitalWrite(LED, HIGH); // openrequest
          }
          else
          {
            digitalWrite(LED, LOW); // closerequest
          }
          break;
        }
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void loop()
{
  switch (currentState)
  {
  case NORMAL:
    xSemaphoreTake(positionMutex, portMAX_DELAY);
    if (lastSentPosition != pendingTargetPosition)
    {
      currentTargetPosition = pendingTargetPosition;
      krs.setPos(0, currentTargetPosition);
      lastSentPosition = currentTargetPosition;
      count_free = 0;
      flag_free = 1;
    }
    xSemaphoreGive(positionMutex);

    if (count > 75)
    {
      count = 0;
      getandsendPos();
    }
    // if (digitalRead(EMG) == HIGH)
    // {
    //   count_EMG++;
    //   if (count_EMG > 3000) // ダンプ試験はここを変える
    //   {
    //     currentState = EMG_ACTIVE;
    //   }
    // }
    // else
    // {
    //   count_EMG = 0;
    // }
    digitalWrite(LED, digitalRead(LED) ^ 1);
    break;
    // case EMG_ACTIVE:
    //   Serial.println("EMG detected, stopping servo.");
    //   if (digitalRead(EMG) == LOW)
    //   {
    //     krs.setPos(0, closePosition);
    //     count_EMG = 0;
    //     currentState = NORMAL;
    //   }
    //   break;
  }
  getandsendPos();
  ++count;
  delay(100);
#ifdef SERIAL_DEBUG
  if (Serial.available())
  {
    int input = Serial.read();
    Serial.print("Input: ");
    Serial.println(input);

    xSemaphoreTake(positionMutex, portMAX_DELAY);
    switch (input)
    {
    case 'o':
      Serial.println("Open position requested");
      pendingTargetPosition = openPosition;
      break;
    case 'c':
      Serial.println("Close position requested");
      pendingTargetPosition = closePosition;
      break;
    case 'f':
      Serial.println("Free requested");
      krs.setFree(0);
      break;
    case 's':
    {
      Serial.println("speed change requested");
      int speed = 127;
      krs.setSpd(0, speed);
      break;
    }
    case 'g':
    {
      Serial.print("servo speed: ");
      int speed = krs.getSpd(0);
      Serial.println(speed);
      break;
    }
    case 't':
    {
      Serial.print("servo tension: ");
      int tension = krs.getStrc(0);
      Serial.println(tension);
    }
    case 'i':
    {
      Serial.print("servo current: ");
      int current = krs.getCur(0);
      Serial.println(current);
    }
    case 'p':
    {
      Serial.print("servo pos: ");
      int position = krs.getPos(0);
      Serial.println(position);
    }
    default:
      break;
    }
    xSemaphoreGive(positionMutex);
  }
  delay(10);
#endif
}
