#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"

#include "../utils/actions.h"

#include "demo_green_side_homologation.h"

void demo_green_side_homologation_task(void * data);

void demo_green_side_homologation_start(struct trajectory_manager *t)
{
  xTaskCreate(demo_green_side_homologation_task, (const signed char *)"DemoGreenSideHomologation", 100, (void *)t, 1, NULL );
}

void demo_green_side_homologation_task(void* data)
{
  struct trajectory_manager *t =(struct trajectory_manager *) data;

  trajectory_goto_d_mm(t, 150);
  while(!trajectory_is_ended(t));
  trajectory_goto_a_rel_deg(t, 45);
  while(!trajectory_is_ended(t));
  trajectory_goto_d_mm(t, 900);
  while(!trajectory_is_ended(t));
  while(1) { vTaskDelay(100 / portTICK_RATE_MS); }
}  
