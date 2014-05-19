#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"

#include "demo_square.h"

void demo_square_task(void *data);

void demo_square_start(struct control_system *am)
{
  xTaskCreate(demo_square_task, (const signed char *)"DemoSquare", 240, (void *)am, 1, NULL );
}

void demo_square_task(void *data)
{
  struct control_system *am = (struct control_system *)data;

  int dist = 0;
  int angl = 0;
  for(;;) {
    dist += 300;
    angl += 90;
    control_system_set_distance_mm_ref(am, dist);
    vTaskDelay(5000 / portTICK_RATE_MS);

    control_system_set_angle_deg_ref(am, angl);
    vTaskDelay(5000 / portTICK_RATE_MS);
  }
}
