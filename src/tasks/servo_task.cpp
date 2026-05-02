#include "tasks.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "../angle_protocol.h"
#include "../app_state.h"

void servo_task(void *pvParameters)
{
  uint8_t error_count = 0;

  while (1)
  {
    bool is_emergency = IS_EMERGENCY.load();
    if (is_emergency)
    {
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
