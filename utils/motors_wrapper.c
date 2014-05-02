#include "motors_wrapper.h"
#include <stdlib.h>

#ifdef UNUSED
#elif defined(__GNUC__)
# define UNUSED(x) UNUSED_ ## x __attribute__((unused))
#elif defined(__LCLINT__)
# define UNUSED(x) /*@unused@*/ x
#else
# define UNUSED(x) x
#endif

struct motors_wrapper mots = {NULL, NULL};

void motors_wrapper_init(struct ausbee_l298_chip *right_motor,
                         struct ausbee_l298_chip *left_motor)
{
  mots.right_motor = right_motor;
  mots.left_motor  = left_motor;
}

void motors_wrapper_right_motor_set_duty_cycle(void *UNUSED(dummy), int32_t duty_cycle)
{
  ausbee_l298_set_duty_cycle(*(mots.right_motor), (uint8_t)duty_cycle);
}

void motors_wrapper_left_motor_set_duty_cycle(void *UNUSED(dummy), int32_t duty_cycle)
{
  ausbee_l298_set_duty_cycle(*(mots.left_motor), (uint8_t)duty_cycle);
}
