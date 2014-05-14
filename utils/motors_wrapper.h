#ifndef MOTORS_WRAPPER_H
#define MOTORS_WRAPPER_H

#include <stdint.h>
#include <AUSBEE/l298_driver.h>

struct motors_wrapper {
  struct ausbee_l298_chip *right_motor, *left_motor;
};

void motors_wrapper_init(struct ausbee_l298_chip *right_motor,
                         struct ausbee_l298_chip *left_motor);
void motors_wrapper_right_motor_set_duty_cycle(void *dummy, float duty_cycle);
void motors_wrapper_left_motor_set_duty_cycle(void *dummy, float duty_cycle);

#endif /* MOTORS_WRAPPER_H */
