#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include <AUSBEE/pid.h>

#include "asserv_manager.h"
#include "trajectory_manager.h"

#define PID_Kp 10
#define PID_Ki 1
#define PID_Kd 2

void control_system_task(void *data);

struct ausbee_pid pid;

void start_control_system(struct ausbee_cs *cs, struct trajectory_manager *traj)
{
  ausbee_init_pid(&pid, PID_Kp, PID_Ki, PID_Kd, 0, 100, 0);

  ausbee_cs_init(cs);
  ausbee_cs_set_controller(cs, ausbee_eval_pid, (void*)&pid);
  ausbee_cs_set_process_command(cs, right_motor_set_duty_cycle , traj);

  xTaskCreate(control_system_task, (const signed char *)"ControlSystem", 1000, (void *)cs, 1, NULL);
}

void control_system_task(void *data)
{
  for (;;) {
    struct ausbee_cs *cs = (struct ausbee_cs *)data;
    ausbee_cs_update(cs);
    vTaskDelay(10 * portTICK_RATE_MS);
  }
}
