#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"

#include "../utils/actions.h"
#include "demo_square_reverse.h"

void demo_square_reverse_task(void *data);

void demo_square_reverse_start(struct trajectory_manager *t)
{
  xTaskCreate(demo_square_reverse_task, (const signed char *)"DemoSquareReverse", 100, (void *)t, 1, NULL );
}

void demo_square_reverse_task(void *data)
{
  struct trajectory_manager *t = (struct trajectory_manager *)data;
  int i = 0;

  while(presence_tirette());

  for(;;) {

    for (i = 0; i < 4; i++) {
      trajectory_goto_d_mm(t, 400);
      while(!trajectory_is_ended(t));
      trajectory_goto_a_rel_deg(t, -90);
      while(!trajectory_is_ended(t));
    }
  }
}
