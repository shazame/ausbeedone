#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"

#include "../utils/actions.h"
#include "demo_square.h"

void demo_square_task(void *data);

void demo_square_start(struct trajectory_manager *t)
{
  xTaskCreate(demo_square_task, (const signed char *)"DemoSquare", 100, (void *)t, 1, NULL );
}

void demo_square_task(void *data)
{
  struct trajectory_manager *t = (struct trajectory_manager *)data;

  while(presence_tirette());

  for(;;) {
    trajectory_goto_d_mm(t, 300);
    trajectory_goto_a_rel_deg(t, 90);
  }
}
