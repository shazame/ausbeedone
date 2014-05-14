/**
 * @file    control_system.c
 * @author  David BITONNEAU <david.bitonneau@gmail.com>
 * @version V1.1
 * @date    13-Mar-2014
 * @brief   A controller system for a two-wheeled robot with encoders.
 *          Implementation file.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

#include "FreeRTOS.h"
#include "task.h"

#include <AUSBEE/pid.h>

#include "utils/position_manager.h"
#include "control_system.h"

#define PID_Kp 10
#define PID_Ki 0
#define PID_Kd 2

void control_system_task(void *data);

void control_system_start(struct control_system *am)
{
  ausbee_init_pid(&(am->pid_right_motor), PID_Kp, PID_Ki, PID_Kd, 0, 50, 0);
  ausbee_init_pid(&(am->pid_left_motor),  PID_Kp, PID_Ki, PID_Kd, 0, 50, 0);

  // Initialise each control system manager
  ausbee_cs_init(&(am->csm_right_motor));
  ausbee_cs_init(&(am->csm_left_motor));

  ausbee_cs_set_measure_fetcher(&(am->csm_right_motor), position_get_right_encoder, NULL);
  ausbee_cs_set_measure_fetcher(&(am->csm_left_motor), position_get_left_encoder, NULL);

  // We use a pid controller because we like it here at Eirbot
  ausbee_cs_set_controller(&(am->csm_right_motor), ausbee_eval_pid, (void*)&(am->pid_right_motor));
  ausbee_cs_set_controller(&(am->csm_left_motor), ausbee_eval_pid, (void*)&(am->pid_left_motor));

  ausbee_cs_set_process_command(&(am->csm_right_motor), motors_wrapper_right_motor_set_duty_cycle, NULL);
  ausbee_cs_set_process_command(&(am->csm_left_motor), motors_wrapper_left_motor_set_duty_cycle, NULL);

  xTaskCreate(control_system_task, (const signed char *)"ControlSystem", 1000, (void *)am, 1, NULL);
}

void control_system_task(void *data)
{
  for (;;) {
    struct control_system *am = (struct control_system *)data;

    ausbee_cs_manage(&(am->csm_right_motor));
    ausbee_cs_manage(&(am->csm_left_motor));

    printf("Right Measure:   %f: 1;"    , (double)ausbee_cs_get_measure(&(am->csm_right_motor)));
    printf("Right Reference: %f: 1;"    , (double)ausbee_cs_get_reference(&(am->csm_right_motor)));
    printf("Right Error:     %f: 1;"    , (double)ausbee_cs_get_error(&(am->csm_right_motor)));
    printf("Right Error sum: %f: 0.1;"  , (double)ausbee_get_pid_error_sum(&(am->pid_right_motor)));
    printf("Right Command:   %f: 10\r\n", (double)ausbee_cs_get_command(&(am->csm_right_motor)));

    printf("Left Measure:   %f: 1;"    , (double)ausbee_cs_get_measure(&(am->csm_left_motor)));
    printf("Left Reference: %f: 1;"    , (double)ausbee_cs_get_reference(&(am->csm_left_motor)));
    printf("Left Error:     %f: 1;"    , (double)ausbee_cs_get_error(&(am->csm_left_motor)));
    printf("Left Error sum: %f: 0.1;"  , (double)ausbee_get_pid_error_sum(&(am->pid_left_motor)));
    printf("Left Command:   %f: 10\r\n", (double)ausbee_cs_get_command(&(am->csm_left_motor)));

    printf("Odometry Distance mm: %f: 1;"   , (double)position_get_distance_mm());
    printf("Odometry Angle:       %f: 1\r\n", (double)position_get_angle_deg());

    printf("Robot x mm: %f;"   , (double)position_get_x_mm());
    printf("Robot y mm: %f\r\n", (double)position_get_y_mm());

    vTaskDelay(1 * portTICK_RATE_MS); // 1 * 100 ms
  }
}

static void control_system_set_motors_ref(struct control_system *am, float d_mm, float theta)
{
  uint32_t axle_track_mm = position_get_axle_track_mm();

  int32_t right_motor_ref = position_mm_to_ticks(d_mm + (1.0 * axle_track_mm * theta) / 2);
  int32_t left_motor_ref  = position_mm_to_ticks(d_mm - (1.0 * axle_track_mm * theta) / 2);

  ausbee_cs_set_reference(&(am->csm_right_motor), right_motor_ref);
  ausbee_cs_set_reference(&(am->csm_left_motor), left_motor_ref);
}

void control_system_set_right_motor_ref(struct control_system *am, int32_t ref)
{
  ausbee_cs_set_reference(&(am->csm_right_motor), ref);
}

void control_system_set_left_motor_ref(struct control_system *am, int32_t ref)
{
  ausbee_cs_set_reference(&(am->csm_left_motor), ref);
}
