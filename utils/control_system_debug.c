#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "control_system_debug.h"

#define PI 3.1415926535

void control_system_debug_task(void *);

void control_system_debug_start(struct control_system *cs)
{
  xTaskCreate(control_system_debug_task, (const signed char *)"ControlSystemDebug", 200, (void *)cs, 1, NULL );
}

void control_system_debug_task(void *data)
{
  struct control_system *cs = (struct control_system *)data;

  for (;;) {
#ifdef USE_MOTOR_SPEED_CS
    printf("Right Measure:          %f: 1;"    , (double)ausbee_cs_get_measure(&(cs->csm_right_motor)));
    printf("Right Filtered measure: %f: 10;"   , (double)ausbee_cs_get_filtered_measure(&(cs->csm_right_motor)));
    printf("Right Reference:        %f: 1;"    , (double)ausbee_cs_get_reference(&(cs->csm_right_motor)));
    printf("Right Error:            %f: 1;"    , (double)ausbee_cs_get_error(&(cs->csm_right_motor)));
    printf("Right Command:          %f: 10\r\n", (double)ausbee_cs_get_command(&(cs->csm_right_motor)));

    printf("Left Measure:          %f: 1;"    , (double)ausbee_cs_get_measure(&(cs->csm_left_motor)));
    printf("Left Filtered measure: %f: 10;"   , (double)ausbee_cs_get_filtered_measure(&(cs->csm_left_motor)));
    printf("Left Reference:        %f: 1;"    , (double)ausbee_cs_get_reference(&(cs->csm_left_motor)));
    printf("Left Error:            %f: 1;"    , (double)ausbee_cs_get_error(&(cs->csm_left_motor)));
    printf("Left Command:          %f: 10\r\n", (double)ausbee_cs_get_command(&(cs->csm_left_motor)));
#endif

#ifdef USE_DISTANCE_ANGLE_CS
    printf("Distance reference mm: %f: 1;"   , (double)ausbee_cs_get_reference(&(cs->csm_distance)));
    printf("Distance filt ref mm:  %f: 1;"   , (double)ausbee_cs_get_filtered_reference(&(cs->csm_distance)));
    printf("Distance measure mm:   %f: 1;"   , (double)ausbee_cs_get_measure(&(cs->csm_distance)));
    printf("Distance error mm:     %f: 1;"   , (double)ausbee_cs_get_error(&(cs->csm_distance)));
    printf("Distance error sum mm: %f: 1;"   , (double)ausbee_pid_get_error_sum(&(cs->pid_distance)));
    printf("Distance error diff mm: %f: 1;"   , (double)ausbee_pid_get_error_diff(&(cs->pid_distance)));
    printf("Distance command mm:   %f: 10\r\n", (double)ausbee_cs_get_command(&(cs->csm_distance)));

    printf("Angle reference deg:   %f: 1;"   , (double)(180.0 / PI * ausbee_cs_get_reference(&(cs->csm_angle))));
    printf("Angle filt ref deg:    %f: 1;"   , (double)(180.0 / PI * ausbee_cs_get_filtered_reference(&(cs->csm_angle))));
    printf("Angle measure deg:     %f: 1;"   , (double)(180.0 / PI * ausbee_cs_get_measure(&(cs->csm_angle))));
    printf("Angle error deg:       %f: 1;"   , (double)(180.0 / PI * ausbee_cs_get_error(&(cs->csm_angle))));
    printf("Angle command deg:     %f: 1\r\n", (double)(180.0 / PI * ausbee_cs_get_command(&(cs->csm_angle))));

    printf("Robot x mm: %f;"   , (double)position_get_x_mm());
    printf("Robot y mm: %f\r\n", (double)position_get_y_mm());
#endif

    vTaskDelay(CONTROL_SYSTEM_PERIOD_S * 1000 / portTICK_RATE_MS);
  }
}
