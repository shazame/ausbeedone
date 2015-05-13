/**
 * @file    control_system.c
 * @author  David BITONNEAU <david.bitonneau@gmail.com>
 * @version V1.2
 * @date    18-Mar-2014
 * @brief   A controller system for a two-wheeled robot with encoders.
 *          Implementation file.
 */

#include <stdint.h>
#include <stdlib.h>
#include <inttypes.h>

#include "FreeRTOS.h"
#include "task.h"

#include <AUSBEE/pid.h>

#include "utils/position_manager.h"
#include "control_system.h"

#define PI 3.1415926535
#define DEG2RAD(a) ((a) * PI / 180.0)

#ifdef UNUSED
#elif defined(__GNUC__)
# define UNUSED(x) UNUSED_ ## x __attribute__((unused))
#elif defined(__LCLINT__)
# define UNUSED(x) /*@unused@*/ x
#else
# define UNUSED(x) x
#endif

// Comment this line to avoid using a control system on motor speed
#define USE_MOTOR_SPEED_CS
// Comment this line to avoid using a control system on distance and angle
#define USE_DISTANCE_ANGLE_CS

void control_system_task(void *data);
#ifdef USE_DISTANCE_ANGLE_CS
static void control_system_set_motors_ref(struct control_system *am, float d_mm, float theta);
static void control_system_set_distance_mm_diff(void *am, float ref);
static void control_system_set_angle_rad_diff(void *am, float ref);
#endif

#ifdef USE_MOTOR_SPEED_CS
static void control_system_init_motors(struct control_system *am)
{
  ausbee_pid_init(&(am->pid_right_motor), 2, 0, 0);
  ausbee_pid_init(&(am->pid_left_motor),  2, 0, 0);

  ausbee_diff_init(&(am->diff_right_motor));
  ausbee_diff_init(&(am->diff_left_motor));

  ausbee_pid_set_output_range(&(am->pid_right_motor), -100, 100);
  ausbee_pid_set_output_range(&(am->pid_left_motor),  -100, 100);

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
#endif

#ifdef USE_DISTANCE_ANGLE_CS
static void control_system_init_distance_angle(struct control_system *am)
{
  ausbee_pid_init(&(am->pid_distance), 0.04, 0.0005, 0);
  ausbee_pid_init(&(am->pid_angle),    0.02, 0.0005, 0);

  ausbee_pid_set_output_range(&(am->pid_distance), -100, 100);
  ausbee_pid_set_output_range(&(am->pid_angle),  -100, 100);

  // Quadramp setup
  ausbee_quadramp_init(&(am->quadramp_distance));
  ausbee_quadramp_init(&(am->quadramp_angle));

  // Setting quadramp eval period to the control system period
  ausbee_quadramp_set_eval_period(&(am->quadramp_distance), CONTROL_SYSTEM_PERIOD_S);
  ausbee_quadramp_set_eval_period(&(am->quadramp_angle),    CONTROL_SYSTEM_PERIOD_S);

  ausbee_quadramp_set_2nd_order_vars(&(am->quadramp_distance),
                                     DISTANCE_MAX_ACC,
                                     DISTANCE_MAX_ACC); // Translation acceleration (in mm/s^2)

  ausbee_quadramp_set_2nd_order_vars(&(am->quadramp_angle),
                                     DEG2RAD(ANGLE_MAX_ACC_DEG),
                                     DEG2RAD(ANGLE_MAX_ACC_DEG)); // Rotation acceleration (in rad/s^2)

  control_system_set_speed_high(am);

  // Initialise each control system manager
  ausbee_cs_init(&(am->csm_distance));
  ausbee_cs_init(&(am->csm_angle));

  // Set reference filter
  ausbee_cs_set_reference_filter(&(am->csm_distance), ausbee_quadramp_eval, (void*)&(am->quadramp_distance));
  ausbee_cs_set_reference_filter(&(am->csm_angle),    ausbee_quadramp_eval, (void*)&(am->quadramp_angle));

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
#endif

void control_system_start(struct control_system *am)
{
#ifdef USE_MOTOR_SPEED_CS
  control_system_init_motors(am);
#endif
#ifdef USE_DISTANCE_ANGLE_CS
  control_system_init_distance_angle(am);
#endif

  xTaskCreate(control_system_task, (const signed char *)"ControlSystem", 200, (void *)am, 1, NULL);
}

void control_system_task(void *data)
{
  for (;;) {
    struct control_system *am = (struct control_system *)data;

#ifdef USE_DISTANCE_ANGLE_CS
    ausbee_cs_manage(&(am->csm_distance));
    ausbee_cs_manage(&(am->csm_angle));

    control_system_set_motors_ref(am, am->distance_mm_diff, am->angle_rad_diff);
#endif

#ifdef USE_MOTOR_SPEED_CS
    ausbee_cs_manage(&(am->csm_right_motor));
    ausbee_cs_manage(&(am->csm_left_motor));
#endif


    vTaskDelay(CONTROL_SYSTEM_PERIOD_S * 1000 / portTICK_RATE_MS);
  }
}

#ifdef USE_DISTANCE_ANGLE_CS
# ifdef USE_MOTOR_SPEED_CS
static void control_system_set_motors_ref(struct control_system *am, float d_mm, float theta)
# else
static void control_system_set_motors_ref(struct control_system *UNUSED(am), float d_mm, float theta)
# endif
{
  uint32_t axle_track_mm = position_get_axle_track_mm();

  int32_t right_motor_ref = position_mm_to_ticks(d_mm + (1.0 * axle_track_mm * theta) / 2);
  int32_t left_motor_ref  = position_mm_to_ticks(d_mm - (1.0 * axle_track_mm * theta) / 2);

# ifdef USE_MOTOR_SPEED_CS
  ausbee_cs_set_reference(&(am->csm_right_motor), right_motor_ref);
  ausbee_cs_set_reference(&(am->csm_left_motor), left_motor_ref);
# else
  motors_wrapper_right_motor_set_duty_cycle(NULL, right_motor_ref);
  motors_wrapper_left_motor_set_duty_cycle(NULL, left_motor_ref);
# endif
}
#endif

#ifdef USE_DISTANCE_ANGLE_CS
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
#endif

// User functions
void control_system_set_distance_mm_ref(struct control_system *am, float ref)
{
  ausbee_cs_set_reference(&(am->csm_distance), ref);
}

void control_system_set_angle_deg_ref(struct control_system *am, float ref_deg)
{
  float ref_rad = DEG2RAD(ref_deg);
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

void control_system_set_distance_max_speed(struct control_system *am, float max_speed)
{
  ausbee_quadramp_set_1st_order_vars(&(am->quadramp_distance), max_speed, max_speed);
}

void control_system_set_distance_max_acc(struct control_system *am, float max_acc)
{
  ausbee_quadramp_set_2nd_order_vars(&(am->quadramp_distance), max_acc, max_acc);
}

void control_system_set_angle_max_speed(struct control_system *am, float max_speed)
{
  ausbee_quadramp_set_1st_order_vars(&(am->quadramp_angle), max_speed, max_speed);
}

void control_system_set_angle_max_acc(struct control_system *am, float max_acc)
{
  ausbee_quadramp_set_2nd_order_vars(&(am->quadramp_angle), max_acc, max_acc);
}

void control_system_set_speed_ratio(struct control_system *am, float ratio)
{
  if (ratio < 0) {
    ratio = 0;
  }
  else if (ratio > 1) {
    ratio = 1;
  }

  control_system_set_distance_max_speed(am, ratio*DISTANCE_MAX_SPEED); // Translation speed (in mm/s)

  control_system_set_angle_max_speed(am, DEG2RAD(ratio*ANGLE_MAX_SPEED_DEG)); // Rotation speed (in rad/s)
}

void control_system_set_speed_high(struct control_system *am)
{
  control_system_set_speed_ratio(am, 1);
}

void control_system_set_speed_medium(struct control_system *am)
{
  control_system_set_speed_ratio(am, 0.7);
}

void control_system_set_speed_low(struct control_system *am)
{
  control_system_set_speed_ratio(am, 0.5);
}
