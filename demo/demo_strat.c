#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"

#include "../utils/actions.h"
#include "demo_strat.h"

void demo_strat_task(void *data);

void demo_strat_start(struct trajectory_manager *t)
{
  xTaskCreate(demo_strat_task, (const signed char *)"DemoStrat", 100, (void *)t, 1, NULL );
}

void demo_strat_task(void *data)
{
  struct trajectory_manager *t = (struct trajectory_manager *)data;

  while(presence_tirette());

  trajectory_goto_d_mm(t, 1000);

  for(;;) {

  }
}
