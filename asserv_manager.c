#include "FreeRTOS.h"
#include "task.h"
#include <AUSBEE/pid.h>
#include "asserv_manager.h"

#define PID_Kp 10
#define PID_Ki 1
#define PID_Kd 2

struct ausbee_pid pid;

void start_control_system(struct ausbee_cs *cs)
{
  ausbee_init_pid(&pid, PID_Kp, PID_Ki, PID_Kd, 0, 100, 0);

  ausbee_cs_init(cs);
  ausbee_cs_set_controller(cs, ausbee_eval_pid, (void*)&pid);

  xTaskCreate(ausbee_cs_update, (const signed char *)"ControlSystem", 100, cs, 1, NULL );
}
