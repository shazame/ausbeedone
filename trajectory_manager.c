#include "trajectory_manager.h"

void set_motors(struct trajectory_manager *traj, struct ausbee_l298_chip *right_motor, struct ausbee_l298_chip *left_motor)
{
  traj->right_motor = right_motor;
  traj->left_motor = left_motor;
}

void right_motor_set_duty_cycle(void *t, int32_t duty_cycle)
{
  struct trajectory_manager *traj = (struct trajectory_manager *)t;
  ausbee_l298_set_duty_cycle(*(traj->right_motor), (uint8_t)duty_cycle);
}

void left_motor_set_duty_cycle(void *t, int32_t duty_cycle)
{
  struct trajectory_manager *traj = (struct trajectory_manager *)t;
  ausbee_l298_set_duty_cycle(*(traj->left_motor), (uint8_t)duty_cycle);
}
