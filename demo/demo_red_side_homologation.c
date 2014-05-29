#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"

#include "../utils/actions.h"

#include "demo_red_side_homologation.h"

void demo_red_side_homologation_task(void * data);

void demo_red_side_homologation_start(struct trajectory_manager *t)
{
  xTaskCreate(demo_red_side_homologation_task, (const signed char *)"DemoYellowSideHomologation", 100, (void *)t, 1, NULL );
}

void demo_red_side_homologation_task(void* data)
{
  struct trajectory_manager *t =(struct trajectory_manager *) data;
  while(presence_tirette())
    ;
  trajectory_goto_d_mm(t, 200);
  while(!trajectory_is_ended(t));
  trajectory_goto_a_rel_deg(t, 45);
  while(!trajectory_is_ended(t));
  trajectory_goto_d_mm(t, 70);
  while(!trajectory_is_ended(t));
  trajectory_goto_a_rel_deg(t, 45);
  while(!trajectory_is_ended(t));
  trajectory_goto_d_mm(t, 620);
  while(!trajectory_is_ended(t));
  while(1);
}  
