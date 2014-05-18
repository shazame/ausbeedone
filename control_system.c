/**
 * @file    control_system.c
 * @author  David BITONNEAU <david.bitonneau@gmail.com>
 * @version V1.2
 * @date    18-Mar-2014
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

#define PI 3.1415926535

void control_system_task(void *data);
static void control_system_set_motors_ref(struct control_system *am, float d_mm, float theta);
static void control_system_set_distance_mm_diff(void *am, float ref);
static void control_system_set_angle_rad_diff(void *am, float ref);

static void control_system_init_motors(struct control_system *am)
{
  ausbee_pid_init(&(am->pid_right_motor), 5, 0, 0);
  ausbee_pid_init(&(am->pid_left_motor),  5, 0, 0);

  ausbee_diff_init(&(am->diff_right_motor));
  ausbee_diff_init(&(am->diff_left_motor));

  ausbee_pid_set_output_range(&(am->pid_right_motor), 0, 50);
  ausbee_pid_set_output_range(&(am->pid_left_motor),  0, 50);

  // Initialise each control system manager
  ausbee_cs_init(&(am->csm_right_motor));
  ausbee_cs_init(&(am->csm_left_motor));

  // Set measure functions
  ausbee_cs_set_measure_fetcher(&(am->csm_right_motor), position_get_right_encoder, NULL);
  ausbee_cs_set_measure_fetcher(&(am->csm_left_motor), position_get_left_encoder, NULL);

  // Set measure filter (asservissement des moteurs en vitesse)
  ausbee_cs_set_measure_filter(&(am->csm_right_motor), ausbee_diff_eval, (void*)&(am->diff_right_motor));
  ausbee_cs_set_measure_filter(&(am->csm_left_motor), ausbee_diff_eval, (void*)&(am->diff_left_motor));

  // We use a pid controller because we like it here at Eirbot
  ausbee_cs_set_controller(&(am->csm_right_motor), ausbee_pid_eval, (void*)&(am->pid_right_motor));
  ausbee_cs_set_controller(&(am->csm_left_motor), ausbee_pid_eval, (void*)&(am->pid_left_motor));

  // Set processing command
  ausbee_cs_set_process_command(&(am->csm_right_motor), motors_wrapper_right_motor_set_duty_cycle, NULL);
  ausbee_cs_set_process_command(&(am->csm_left_motor), motors_wrapper_left_motor_set_duty_cycle, NULL);
}

static void control_system_init_distance_angle(struct control_system *am)
{
  ausbee_pid_init(&(am->pid_distance), 1, 0, 0);
  ausbee_pid_init(&(am->pid_angle),    1, 0, 0);

  // Initialise each control system manager
  ausbee_cs_init(&(am->csm_distance));
  ausbee_cs_init(&(am->csm_angle));

  // Set measure functions
  ausbee_cs_set_measure_fetcher(&(am->csm_distance), position_get_distance_mm, NULL);
  ausbee_cs_set_measure_fetcher(&(am->csm_angle), position_get_angle_rad, NULL);

  // We use a pid controller because we like it here at Eirbot
  ausbee_cs_set_controller(&(am->csm_distance), ausbee_pid_eval, (void*)&(am->pid_distance));
  ausbee_cs_set_controller(&(am->csm_angle), ausbee_pid_eval, (void*)&(am->pid_angle));

  // Set processing command
  ausbee_cs_set_process_command(&(am->csm_distance), control_system_set_distance_mm_diff, am);
  ausbee_cs_set_process_command(&(am->csm_angle),    control_system_set_angle_rad_diff,   am);

  control_system_set_distance_mm_diff(am, 0);
  control_system_set_angle_rad_diff(am, 0);
}

void control_system_start(struct control_system *am)
{
  control_system_init_motors(am);
  control_system_init_distance_angle(am);

  xTaskCreate(control_system_task, (const signed char *)"ControlSystem", 1000, (void *)am, 1, NULL);
}

void control_system_task(void *data)
{
  for (;;) {
    struct control_system *am = (struct control_system *)data;

    ausbee_cs_manage(&(am->csm_distance));
    ausbee_cs_manage(&(am->csm_angle));

    control_system_set_motors_ref(am, am->distance_mm_diff, am->angle_rad_diff);

    ausbee_cs_manage(&(am->csm_right_motor));
    ausbee_cs_manage(&(am->csm_left_motor));

    printf("Right Measure:          %f: 1;"    , (double)ausbee_cs_get_measure(&(am->csm_right_motor)));
    printf("Right Filtered measure: %f: 10;"   , (double)ausbee_cs_get_filtered_measure(&(am->csm_right_motor)));
    printf("Right Reference:        %f: 1;"    , (double)ausbee_cs_get_reference(&(am->csm_right_motor)));
    printf("Right Error:            %f: 1;"    , (double)ausbee_cs_get_error(&(am->csm_right_motor)));
    printf("Right Command:          %f: 10\r\n", (double)ausbee_cs_get_command(&(am->csm_right_motor)));

    printf("Left Measure:          %f: 1;"    , (double)ausbee_cs_get_measure(&(am->csm_left_motor)));
    printf("Left Filtered measure: %f: 10;"   , (double)ausbee_cs_get_filtered_measure(&(am->csm_left_motor)));
    printf("Left Reference:        %f: 1;"    , (double)ausbee_cs_get_reference(&(am->csm_left_motor)));
    printf("Left Error:            %f: 1;"    , (double)ausbee_cs_get_error(&(am->csm_left_motor)));
    printf("Left Command:          %f: 10\r\n", (double)ausbee_cs_get_command(&(am->csm_left_motor)));

    //printf("Distance reference mm: %f: 1;"   , (double)ausbee_cs_get_reference(&(am->csm_distance)));
    //printf("Distance measure mm:   %f: 1;"   , (double)ausbee_cs_get_measure(&(am->csm_distance)));
    //printf("Distance error mm:     %f: 1;"   , (double)ausbee_cs_get_error(&(am->csm_distance)));
    //printf("Distance command mm:   %f: 1\r\n", (double)ausbee_cs_get_command(&(am->csm_distance)));

    printf("Angle reference rad:   %f: 1;"   , (double)ausbee_cs_get_reference(&(am->csm_angle)));
    printf("Angle measure rad:     %f: 1;"   , (double)ausbee_cs_get_measure(&(am->csm_angle)));
    printf("Angle error rad:       %f: 1;"   , (double)ausbee_cs_get_error(&(am->csm_angle)));
    printf("Angle command rad:     %f: 1\r\n", (double)ausbee_cs_get_command(&(am->csm_angle)));

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

static void control_system_set_distance_mm_diff(void *data, float ref)
{
  struct control_system *am = (struct control_system *)data;
  am->distance_mm_diff = ref;
}

static void control_system_set_angle_rad_diff(void *data, float ref)
{
  struct control_system *am = (struct control_system *)data;
  am->angle_rad_diff = ref;
}

// User functions
void control_system_set_distance_mm_ref(struct control_system *am, float ref)
{
  ausbee_cs_set_reference(&(am->csm_distance), ref);
}

void control_system_set_angle_deg_ref(struct control_system *am, float ref_deg)
{
  float ref_rad = ref_deg * PI / 180.0;
  ausbee_cs_set_reference(&(am->csm_angle), ref_rad);
}

void control_system_set_right_motor_ref(struct control_system *am, int32_t ref)
{
  ausbee_cs_set_reference(&(am->csm_right_motor), ref);
}

void control_system_set_left_motor_ref(struct control_system *am, int32_t ref)
{
  ausbee_cs_set_reference(&(am->csm_left_motor), ref);
}
