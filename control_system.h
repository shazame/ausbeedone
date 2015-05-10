/**
 * @file    control_system.h
 * @author  David BITONNEAU <david.bitonneau@gmail.com>
 * @version V1.0
 * @date    18-Mar-2014
 * @brief   A controller system for a two-wheeled robot with encoders.
 *          Definition file.
 */

#ifndef CONTROL_SYSTEM_H
#define CONTROL_SYSTEM_H

#include <AUSBEE/control_system_manager.h>
#include <AUSBEE/pid.h>
#include <AUSBEE/diff.h>
#include <AUSBEE/quadramp.h>

#include "utils/motors_wrapper.h"

#define CONTROL_SYSTEM_PERIOD_S 0.1 // 100 ms

#define DISTANCE_MAX_SPEED 700 // in mm/s
#define DISTANCE_MAX_ACC   350 // in mm/s^2

#define ANGLE_MAX_SPEED_DEG 180 // in deg/s
#define ANGLE_MAX_ACC_DEG   100 // in deg/s^2

struct control_system {
  struct ausbee_cs csm_right_motor;
  struct ausbee_cs csm_left_motor;
  struct ausbee_cs csm_distance;
  struct ausbee_cs csm_angle;

  struct ausbee_diff diff_right_motor;
  struct ausbee_diff diff_left_motor;

  struct ausbee_pid pid_right_motor;
  struct ausbee_pid pid_left_motor;
  struct ausbee_pid pid_distance;
  struct ausbee_pid pid_angle;

  struct ausbee_quadramp quadramp_distance;
  struct ausbee_quadramp quadramp_angle;

  float distance_mm_diff;
  float angle_rad_diff;
};

void control_system_start(struct control_system *);

void control_system_set_distance_mm_ref(struct control_system *am, float ref);
void control_system_set_angle_deg_ref(struct control_system *am, float ref);
void control_system_set_right_motor_ref(struct control_system *am, int32_t ref);
void control_system_set_left_motor_ref(struct control_system *am, int32_t ref);

void control_system_set_distance_max_speed(struct control_system *am, float max_speed);
void control_system_set_distance_max_acc(struct control_system *am, float max_acc);
void control_system_set_angle_max_speed(struct control_system *am, float max_speed);
void control_system_set_angle_max_acc(struct control_system *am, float max_acc);

void control_system_set_speed_ratio(struct control_system *am, float ratio);
void control_system_set_speed_high(struct control_system *am);
void control_system_set_speed_medium(struct control_system *am);
void control_system_set_speed_low(struct control_system *am);

#endif /* CONTROL_SYSTEM_H */
