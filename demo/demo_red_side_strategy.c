#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"

#include "../utils/actions.h"

#include "demo_red_side_strategy.h"

void demo_red_side_strategy_task(void * data);

void demo_red_side_strategy_start(struct trajectory_manager *t)
{
  xTaskCreate(demo_red_side_strategy_task, (const signed char *)"DemoYellowSideHomologation", 200, (void *)t, 1, NULL );
}

void demo_red_side_strategy_task(void* data)
{
  struct trajectory_manager *t =(struct trajectory_manager *) data;
  while(presence_tirette())
    ;
  enable_turbine();
  trajectory_goto_d_mm(t, 150);
  trajectory_goto_a_rel_deg(t, 45);
  while(!trajectory_is_ended(t))
    ;
  ouvrir_bras_droit();
  vTaskDelay(50);
  trajectory_goto_a_rel_deg(t, -45);
  trajectory_goto_d_mm(t, 210);
  trajectory_goto_a_rel_deg(t, 25);
  while(!trajectory_is_ended(t))
    ;
  trajectory_goto_d_mm(t, 400);
  fermer_bras_droit();
  trajectory_goto_a_rel_deg(t, 55);
  trajectory_goto_d_mm(t, 100);
  while(!trajectory_is_ended(t))
    ;
  ouvrir_bras_droit();
  vTaskDelay(75);
  fermer_bras_droit();
  vTaskDelay(75);
  trajectory_goto_a_rel_deg(t, 80);
  trajectory_goto_d_mm(t, 400);
  //TODO : Add lauch ball
  trajectory_goto_a_rel_deg(t, -80);
  trajectory_goto_d_mm(t, 400);
  trajectory_goto_a_rel_deg(t, 90);
  trajectory_goto_d_mm(t, 200);
  trajectory_goto_a_rel_deg(t, 180);
  //TODO: add remove from stack
  //while(contact_fresque)
  trajectory_goto_d_mm(t, -1000);
  while(!trajectory_is_ended(t))
    ;
  placer_peinture_ausbee();
  placer_peinture_canon();
  vTaskDelay(100);
  trajectory_goto_d_mm(t, 500);  
}
  


