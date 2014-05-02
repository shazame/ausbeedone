#include <stdint.h>
#include <stdio.h>
#include <inttypes.h>

#include "FreeRTOS.h"
#include "task.h"

#include <AUSBEE/pid.h>

#include "utils/encoders.h"
#include "asserv_manager.h"

#define PID_Kp 10
#define PID_Ki 0
#define PID_Kd 2

void control_system_task(void *data);

struct asserv_manager am;

void start_control_system(struct asserv_manager *am, struct trajectory_manager *traj)
{
  ausbee_init_pid(&(am->pid_right_motor), PID_Kp, PID_Ki, PID_Kd, 100, 0);
  ausbee_init_pid(&(am->pid_left_motor), PID_Kp, PID_Ki, PID_Kd, 100, 0);

  // Initialise each control system manager
  ausbee_cs_init(&(am->csm_right_motor));
  ausbee_cs_init(&(am->csm_left_motor));

  // We use a pid controller because we like it here at Eirbot
  ausbee_cs_set_controller(&(am->csm_right_motor), ausbee_eval_pid, (void*)&(am->pid_right_motor));
  ausbee_cs_set_controller(&(am->csm_left_motor), ausbee_eval_pid, (void*)&(am->pid_left_motor));

  ausbee_cs_set_process_command(&(am->csm_right_motor), right_motor_set_duty_cycle , traj);
  ausbee_cs_set_process_command(&(am->csm_left_motor), left_motor_set_duty_cycle , traj);

  xTaskCreate(control_system_task, (const signed char *)"ControlSystem", 1000, (void *)am, 1, NULL);
}

void control_system_task(void *data)
{
  for (;;) {
    struct asserv_manager *am = (struct asserv_manager *)data;

    int32_t right_motor_command = ausbee_cs_update(&(am->csm_right_motor), get_right_encoder_value());
    ausbee_cs_update(&(am->csm_left_motor), get_left_encoder_value());

    printf("Measure: %"PRId32": 1,", get_right_encoder_value());
    printf("Error: %"PRId32": 1,", ausbee_get_pid_error(&(am->pid_right_motor)));
    printf("Error sum: %"PRId32": 0.1,", ausbee_get_pid_error_sum(&(am->pid_right_motor)));
    printf("Command: %"PRId32": 10\r\n", right_motor_command);

    vTaskDelay(1 * portTICK_RATE_MS); // 1 * 100 ms
  }
}

void control_system_set_right_motor_ref(struct asserv_manager *am, int32_t ref)
{
  ausbee_pid_set_ref(&(am->pid_right_motor), ref);
}
