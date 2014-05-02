#include "motors_wrapper.h"
#include <stdlib.h>

void motors_wrapper_init(struct motors_wrapper   *mots,
                         struct ausbee_l298_chip *right_motor,
                         struct ausbee_l298_chip *left_motor)
{
  mots->right_motor = right_motor;
  mots->left_motor  = left_motor;
}

void motors_wrapper_right_motor_set_duty_cycle(void *mots, int32_t duty_cycle)
{
  struct motors_wrapper *motors = (struct motors_wrapper *)mots;
  ausbee_l298_set_duty_cycle(*(motors->right_motor), (uint8_t)duty_cycle);
}

void motors_wrapper_left_motor_set_duty_cycle(void *mots, int32_t duty_cycle)
{
  struct motors_wrapper *motors = (struct motors_wrapper *)mots;
  ausbee_l298_set_duty_cycle(*(motors->left_motor), (uint8_t)duty_cycle);
}
