#include "motors_wrapper.h"
#include <stdlib.h>
#include <stdio.h>

#ifdef UNUSED
#elif defined(__GNUC__)
# define UNUSED(x) UNUSED_ ## x __attribute__((unused))
#elif defined(__LCLINT__)
# define UNUSED(x) /*@unused@*/ x
#else
# define UNUSED(x) x
#endif

struct motors_wrapper mots = {NULL, NULL, 0, 0};

void motors_wrapper_init(struct ausbee_l298_chip *left_motor,
                         struct ausbee_l298_chip *right_motor)
{
  mots.left_motor  = left_motor;
  mots.right_motor = right_motor;

  mots.right_motor_moving_forward = 1;
  mots.left_motor_moving_forward  = 1;
}

static void motors_wrapper_motor_set_duty_cycle(struct ausbee_l298_chip *motor, float duty_cycle, uint8_t *motor_moving_forward)
{
  if (motor == NULL) {
    printf("[motors_wrapper] Error: Motors were not set properly.\n");
    return;
  }

  // Moving backward
  if (duty_cycle < 0) {
    *motor_moving_forward = 0;
    ausbee_l298_invert_output(*motor, 1);
    ausbee_l298_set_duty_cycle(*motor, (uint8_t)-duty_cycle);
  }
  // Moving forward
  else {
    *motor_moving_forward = 1;
    ausbee_l298_invert_output(*motor, 0);
    ausbee_l298_set_duty_cycle(*motor, (uint8_t)duty_cycle);
  }
}

void motors_wrapper_right_motor_set_duty_cycle(void *UNUSED(dummy), float duty_cycle)
{
  motors_wrapper_motor_set_duty_cycle((mots.right_motor), duty_cycle, &(mots.right_motor_moving_forward));
}

void motors_wrapper_left_motor_set_duty_cycle(void *UNUSED(dummy), float duty_cycle)
{
  motors_wrapper_motor_set_duty_cycle((mots.left_motor), duty_cycle, &(mots.left_motor_moving_forward));
}

uint8_t motors_wrapper_right_motor_is_moving_forward(void)
{
  return mots.right_motor_moving_forward;
}

uint8_t motors_wrapper_left_motor_is_moving_forward(void)
{
  return mots.left_motor_moving_forward;
}
