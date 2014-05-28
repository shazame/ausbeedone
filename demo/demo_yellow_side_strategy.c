#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"

#include "../utils/actions.h"

#include "demo_yellow_side_strategy.h"

void demo_yellow_side_strategy_task(void * data);

void demo_yellow_side_strategy_start(struct trajectory_manager *t)
{
  xTaskCreate(demo_yellow_side_strategy_task, (const signed char *)"DemoYellowSideHomologation", 200, (void *)t, 1, NULL );
}

void demo_yellow_side_strategy_task(void* data)
{
  struct trajectory_manager *t =(struct trajectory_manager *) data;
  while(presence_tirette())
    ;
  enable_turbine();
  trajectory_goto_d_mm(t, 150);
  while(!trajectory_is_ended(t));
  trajectory_goto_a_rel_deg(t, -45);
  while(!trajectory_is_ended(t))
    ;
  ouvrir_bras_gauche();
  vTaskDelay(50);
  trajectory_goto_a_rel_deg(t, 45);
  while(!trajectory_is_ended(t));
  trajectory_goto_d_mm(t, 120);
  while(!trajectory_is_ended(t));
  trajectory_goto_a_rel_deg(t, -30);
  while(!trajectory_is_ended(t))
    ;
  trajectory_goto_d_mm(t, 250);
  vTaskDelay(150);
  fermer_bras_gauche();
  while(!trajectory_is_ended(t));
  trajectory_goto_a_rel_deg(t, -60);
  while(!trajectory_is_ended(t));
  trajectory_goto_d_mm(t, 100);
  while(!trajectory_is_ended(t))
    ;
  ouvrir_bras_gauche();
  vTaskDelay(75);
  fermer_bras_gauche();
  vTaskDelay(75);
  trajectory_goto_a_rel_deg(t, -80);
  while(!trajectory_is_ended(t));
  trajectory_goto_d_mm(t, 280);
  while(!trajectory_is_ended(t));
  trajectory_goto_a_rel_deg(t, 80);
  while(!trajectory_is_ended(t));
  trajectory_goto_d_mm(t, 250);
  while(!trajectory_is_ended(t));
  trajectory_goto_a_rel_deg(t, 90);
  while(!trajectory_is_ended(t));
  lancer_une_balle();
  //vTaskDelay(400);
  trajectory_goto_a_rel_deg(t, 10);
  while(!trajectory_is_ended(t));
  lancer_une_balle();
  //vTaskDelay(400);
  trajectory_goto_a_rel_deg(t, -100);
  while(!trajectory_is_ended(t));
  trajectory_goto_d_mm(t, 800);
  while(!trajectory_is_ended(t));
  trajectory_goto_a_rel_deg(t, -90);
  while(!trajectory_is_ended(t));
  trajectory_goto_d_mm(t, 200);
  while(!trajectory_is_ended(t));
  trajectory_goto_a_rel_deg(t, -180);
  while(!trajectory_is_ended(t));
  //TODO: add remove from stack
  //while(contact_fresque)
  trajectory_goto_d_mm(t, -1000);
  while(contact_fresque()==0)
    ;
  placer_peinture_ausbee();
  placer_peinture_canon();
  vTaskDelay(100);
  trajectory_goto_d_mm(t, 500);  
  while(!trajectory_is_ended(t));
}

