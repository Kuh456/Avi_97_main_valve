#include "tasks.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "../angle_protocol.h"
#include "../app_state.h"

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
