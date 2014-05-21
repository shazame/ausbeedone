#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"

#include "demo_square.h"

void demo_square_task(void *data);

void demo_square_start(struct trajectory_manager *t)
{
  xTaskCreate(demo_square_task, (const signed char *)"DemoSquare", 100, (void *)t, 1, NULL );
}

void demo_square_task(void *data)
{
  struct trajectory_manager *t = (struct trajectory_manager *)data;

  for(;;) {
    trajectory_goto_d_mm(t, 300);
    vTaskDelay(5000 / portTICK_RATE_MS);

    trajectory_goto_a_rel_deg(t, 90);
    vTaskDelay(5000 / portTICK_RATE_MS);
  }
}
