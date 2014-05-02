#ifndef TRAJECTORY_MANAGER_H
#define TRAJECTORY_MANAGER_H

#include <stdint.h>
#include <AUSBEE/l298_driver.h>

struct trajectory_manager {
  struct ausbee_l298_chip *right_motor, *left_motor;
};

void set_motors(struct trajectory_manager *traj, struct ausbee_l298_chip *right_motor, struct ausbee_l298_chip *left_motor);
void right_motor_set_duty_cycle(void *, int32_t duty_cycle);
void left_motor_set_duty_cycle(void *, int32_t duty_cycle);

#endif /* TRAJECTORY_MANAGER_H */
