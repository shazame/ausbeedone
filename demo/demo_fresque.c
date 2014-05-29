#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"

#include "../utils/actions.h"
#include "../utils/lidar_detect.h"
#include "demo_fresque.h"

void demo_fresque_task(void *data);

void demo_fresque_start(struct trajectory_manager *t)
{
  xTaskCreate(demo_fresque_task, (const signed char *)"DemoFresque", 100, (void *)t, 1, NULL );
}

void demo_fresque_task(void *data)
{
  struct trajectory_manager *t = (struct trajectory_manager *)data;

  while(presence_tirette());

  lidar_detect_disable();
  trajectory_goto_d_mm(t, -1000);
  while(!trajectory_is_ended(t)) {
    if (contact_fresque()) {
      trajectory_next_point(t);
      break;
    }
  }
  lidar_detect_enable();
  trajectory_goto_d_mm(t, 500);

  for(;;) {

  }
}
