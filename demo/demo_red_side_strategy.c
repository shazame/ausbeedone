#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"

#include "../utils/actions.h"

#include "demo_red_side_strategy.h"

extern volatile uint8_t enable_detection;

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
  while(!trajectory_is_ended(t));
  trajectory_goto_d_mm(t, 120);
  while(!trajectory_is_ended(t));
  trajectory_goto_a_rel_deg(t, 30);
  while(!trajectory_is_ended(t))
    ;
  trajectory_goto_d_mm(t, 250);
  vTaskDelay(150);
  fermer_bras_droit();
  while(!trajectory_is_ended(t));
  trajectory_goto_a_rel_deg(t, 60);
  while(!trajectory_is_ended(t));
  trajectory_goto_d_mm(t, 100);
  while(!trajectory_is_ended(t))
    ;
  ouvrir_bras_droit();
  vTaskDelay(75);
  fermer_bras_droit();
  vTaskDelay(75);
  trajectory_goto_a_rel_deg(t, 80);
  while(!trajectory_is_ended(t));
  trajectory_goto_d_mm(t, 280);
  while(!trajectory_is_ended(t));
  trajectory_goto_a_rel_deg(t, -80);
  while(!trajectory_is_ended(t));
  trajectory_goto_d_mm(t, 220);
  while(!trajectory_is_ended(t));
  trajectory_goto_a_rel_deg(t, 90);
  while(!trajectory_is_ended(t));
  lancer_une_balle();
  trajectory_goto_a_rel_deg(t, 8);
  while(!trajectory_is_ended(t));
  lancer_une_balle();
  trajectory_goto_a_rel_deg(t, 8);
  while(!trajectory_is_ended(t));
  lancer_une_balle();
  trajectory_goto_a_rel_deg(t, 8);
  while(!trajectory_is_ended(t));
  lancer_une_balle();
  trajectory_goto_d_mm(t, 50);
  while(!trajectory_is_ended(t));
  trajectory_goto_a_rel_deg(t, -12);
  while(!trajectory_is_ended(t));
  lancer_une_balle();
  trajectory_goto_a_rel_deg(t, -16);
  while(!trajectory_is_ended(t));
  lancer_une_balle();
  trajectory_goto_a_rel_deg(t, -84);
  while(!trajectory_is_ended(t));
  trajectory_goto_d_mm(t, 650);
  while(!trajectory_is_ended(t));
  trajectory_goto_a_rel_deg(t, 90);
  while(!trajectory_is_ended(t));
  trajectory_goto_d_mm(t, 100);
  while(!trajectory_is_ended(t));
  //disable detection
  enable_detection=0;
  trajectory_goto_d_mm(t, 100);
  while(!trajectory_is_ended(t));
  trajectory_goto_a_rel_deg(t, 180);
  while(!trajectory_is_ended(t));
  enable_detection=1;
  //TODO: add remove from stack
  //while(contact_fresque)
  trajectory_goto_d_mm(t, -1000);
  while(contact_fresque()==0)
    ;
  trajectory_next_point(t);
  placer_peinture_ausbee();
  placer_peinture_canon();
  vTaskDelay(100);
  trajectory_goto_d_mm(t, 500);  
  while(!trajectory_is_ended(t));
}
  


