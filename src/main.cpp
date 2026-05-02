#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "app_state.h"
#include "tasks/tasks.h"

unsigned long last_panel_rx = 0;
unsigned long last_main_rx = 0;
unsigned long next_blink_toggle = 0;
bool is_blink_high = false;

void setup()
{
  Serial.begin(115200);

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

  Serial1.begin(BAUDRATE, SERIAL_8N1, RX_MAIN_VALVE, TX_MAIN_VALVE);
  krs.begin();

  Serial.println("Ready for ICS Servo");

  xTaskCreatePinnedToCore(can_receive_task, "CAN_Rx", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(can_transmit_task, "CAN_Tx", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(servo_task, "Servo", 4096, NULL, 2, NULL, 1);
}

void loop()
{
  unsigned long now = millis();

  if (PANEL_RX_FLAG.exchange(false))
    last_panel_rx = now;
  if (MAIN_RX_FLAG.exchange(false))
    last_main_rx = now;

  bool panel_alive = (now - last_panel_rx) < COMMUNICATION_TIMEOUT_MS;
  bool main_alive = (now - last_main_rx) < COMMUNICATION_TIMEOUT_MS;
  bool is_timeout = !panel_alive || !main_alive;

  if (now >= next_blink_toggle)
  {
    is_blink_high = !is_blink_high;
    next_blink_toggle = now + 300;
  }

  bool is_servo_error = SERVO_ERROR.load();
  uint8_t main_state = MAIN_STATE.load();

  if (!panel_alive)
  {
    digitalWrite(CAN_LED, is_blink_high ? HIGH : LOW);
  }
  else
  {
    digitalWrite(CAN_LED, HIGH);
  }

  if (!main_alive)
  {
    digitalWrite(MAIN_LED, is_blink_high ? HIGH : LOW);
  }
  else
  {
    if (main_state == 0)
      digitalWrite(MAIN_LED, HIGH);
    else if (main_state == 2)
      digitalWrite(MAIN_LED, is_blink_high ? HIGH : LOW);
    else
      digitalWrite(MAIN_LED, LOW);
  }

  IS_EMERGENCY.store(digitalRead(EMG_PIN) == LOW);

  uint8_t current_state;
  if (is_timeout)
  {
    current_state = 2;
  }
  else if (is_servo_error)
  {
    current_state = 1;
  }
  else
  {
    current_state = 0;
  }

  STATE_TO_CTRL.store(current_state);

  if (current_state == 0)
  {
    digitalWrite(SERVO_LED, HIGH);
  }
  else if (current_state == 1)
  {
    digitalWrite(SERVO_LED, is_blink_high ? HIGH : LOW);
  }
  else
  {
    digitalWrite(SERVO_LED, LOW);
  }

  delay(100);
}
