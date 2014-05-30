#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"

#include "../utils/actions.h"
#include "demo_pid.h"

static float ak = 0;

static xTaskHandle xHandle = NULL;

void demo_pid_task(void *data);
void demo_pid_test_angles(struct trajectory_manager *t);
void demo_pid_test_distance_and_angle(struct trajectory_manager *t);

void demo_pid_start(struct trajectory_manager *t, float angle_coeff)
{
  ak = angle_coeff;
  xTaskCreate(demo_pid_task, (const signed char *)"DemoPID", 100, (void *)t, 1, &xHandle);
}

void demo_pid_stop(void)
{
  if (xHandle != NULL) {
    vTaskDelete(xHandle);
  }
  xHandle = NULL;
}

void demo_pid_task(void *data)
{
  struct trajectory_manager *t = (struct trajectory_manager *)data;

  while(presence_tirette());

  demo_pid_test_angles(t);
}

void demo_pid_test_angle(struct trajectory_manager *t, float a)
{
  trajectory_goto_a_rel_deg(t, a);
  while(!trajectory_is_ended(t));
  trajectory_goto_a_rel_deg(t, -a);
  while(!trajectory_is_ended(t));
}

void demo_pid_test_angles(struct trajectory_manager *t)
{
  demo_pid_test_angle(t, 10);
  demo_pid_test_angle(t, 20);
  demo_pid_test_angle(t, 45);
  demo_pid_test_angle(t, 90);

  for(;;) {
  }
}

void demo_pid_test_distance_and_angle(struct trajectory_manager *t)
{
  trajectory_goto_d_mm(t, 300);
  while(!trajectory_is_ended(t));
  trajectory_goto_a_rel_deg(t, ak*45);
  while(!trajectory_is_ended(t));
  trajectory_goto_a_rel_deg(t, ak*45);
  while(!trajectory_is_ended(t));
  trajectory_goto_d_mm(t, 100);
  while(!trajectory_is_ended(t));
  trajectory_goto_a_rel_deg(t, ak*30);
  while(!trajectory_is_ended(t));
  trajectory_goto_a_rel_deg(t, ak*30);
  while(!trajectory_is_ended(t));
  trajectory_goto_a_rel_deg(t, ak*30);
  while(!trajectory_is_ended(t));
  trajectory_goto_d_mm(t, 100);
  while(!trajectory_is_ended(t));
  trajectory_goto_a_rel_deg(t, ak*90);
  while(!trajectory_is_ended(t));
  trajectory_goto_d_mm(t, 500);
  while(!trajectory_is_ended(t));
  trajectory_goto_a_rel_deg(t, ak*12);
  while(!trajectory_is_ended(t));
  trajectory_goto_a_rel_deg(t, ak*10);
  while(!trajectory_is_ended(t));
  trajectory_goto_a_rel_deg(t, ak*8);
  while(!trajectory_is_ended(t));
  trajectory_goto_a_rel_deg(t, ak*15);
  while(!trajectory_is_ended(t));
  trajectory_goto_a_rel_deg(t, ak*45);
  while(!trajectory_is_ended(t));
  trajectory_goto_a_rel_deg(t, ak*90);
  while(!trajectory_is_ended(t));
  trajectory_goto_d_mm(t, 400);
  while(!trajectory_is_ended(t));
  trajectory_goto_a_rel_deg(t, ak*90);
  while(!trajectory_is_ended(t));
  trajectory_goto_d_mm(t, 200);
  while(!trajectory_is_ended(t));
  trajectory_goto_a_rel_deg(t, ak*180);
  while(!trajectory_is_ended(t));
}
