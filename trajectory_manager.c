#include "FreeRTOS.h"
#include "task.h"

#include "utils/position_manager.h"
#include "trajectory_manager.h"

void trajectory_task(void *data);

void trajectory_init(struct trajectory_manager *t,
                     struct control_system *cs)
{
  t->cs = cs;
}

void trajectory_start(struct trajectory_manager *t)
{
  //xTaskCreate(trajectory_task, (const signed char *)"TrajectoryManager", 240, (void *)t, 1, NULL );
}

void trajectory_task(void *data)
{
  struct trajectory_manager *t = (struct trajectory_manager *)data;

  for (;;) {
    // TODO
    vTaskDelay(1000 / portTICK_RATE_MS);
  }
}

void trajectory_goto_d_mm(struct trajectory_manager *t, float d_mm)
{
  float d_mm_ref = position_get_distance_mm(NULL) + d_mm;

  control_system_set_distance_mm_ref(t->cs, d_mm_ref);
}

/* Absolute angle = current angle + relative angle. */
void trajectory_goto_a_abs_deg(struct trajectory_manager *t,
                               float a_deg_ref)
{
  control_system_set_angle_deg_ref(t->cs, a_deg_ref);
}

void trajectory_goto_a_rel_deg(struct trajectory_manager *t,
                               float a_deg)
{
  float a_deg_ref = position_get_angle_deg(NULL) + a_deg;

  control_system_set_angle_deg_ref(t->cs, a_deg_ref);
}
