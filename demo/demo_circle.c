#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"

#include "demo_circle.h"

void demo_circle_task(void *data);

void demo_circle_start(struct trajectory_manager *t)
{
  xTaskCreate(demo_circle_task, (const signed char *)"DemoCircle", 100, (void *)t, 1, NULL );
}

void demo_circle_task(void *data)
{
  struct trajectory_manager *t = (struct trajectory_manager *)data;

  for(;;) {
    trajectory_goto_d_mm(t, 800);
    trajectory_goto_a_rel_deg(t, 360);
    vTaskDelay(15000 / portTICK_RATE_MS);
  }
}
