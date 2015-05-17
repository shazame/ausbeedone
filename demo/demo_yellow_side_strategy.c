#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"

#include "../utils/servo.h"

#include "demo_yellow_side_strategy.h"

extern volatile uint8_t enable_detection;

static xTaskHandle xHandle = NULL;

void demo_yellow_side_strategy_task(void * data);

void demo_yellow_side_strategy_start(struct trajectory_manager *t)
{
  xTaskCreate(demo_yellow_side_strategy_task, (const signed char *)"DemoYellowSideHomologation", 200, (void *)t, 1, &xHandle);
}

void demo_yellow_side_strategy_stop(void)
{
  if (xHandle != NULL) {
    vTaskDelete(xHandle);
  }
  xHandle = NULL;
}

void demo_yellow_side_strategy_task(void* data)
{
  struct trajectory_manager *t =(struct trajectory_manager *) data;

  trajectory_goto_d_mm(t, 150);
  while(!trajectory_is_ended(t));
  trajectory_goto_a_rel_deg(t, -45);
  while(!trajectory_is_ended(t));
  trajectory_goto_d_mm(t, 900);
  while(!trajectory_is_ended(t));
  trajectory_goto_a_rel_deg(t,45);
  while(!trajectory_is_ended(t));
  trajectory_goto_d_mm(t,-850);// Recul, recalage?
  while(!trajectory_is_ended(t));
  servo_right_arm_open();
  trajectory_goto_d_mm(t,845);
  while(!trajectory_is_ended(t));
  servo_right_arm_close();

  while(1) { vTaskDelay(100 / portTICK_RATE_MS); }
}
