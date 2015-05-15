#include <AUSBEE/servo.h>

#include "servo.h"

static ausbeeServo servo_left_arm, servo_right_arm;

void servo_init_starting_position(void)
{
  ausbeeInitStructServo(&servo_left_arm, 30, 105, TIM11, TIM_Channel_1);
  ausbeeInitStructServo(&servo_right_arm, 30, 105, TIM13, TIM_Channel_1);

  ausbeeInitServo(&servo_left_arm);
  ausbeeInitServo(&servo_right_arm);

  servo_left_arm_close();
  servo_right_arm_close();
}

// Left arm servo
uint8_t servo_left_arm_get_angle(void)
{
  return ausbeeGetAngleServo(&servo_left_arm);
}

void servo_left_arm_set_angle(uint8_t angle)
{
  ausbeeSetAngleServo(&servo_left_arm, angle);
}

void servo_left_arm_open(void)
{
  servo_left_arm_set_angle(90);
}

void servo_left_arm_close(void)
{
  servo_left_arm_set_angle(30);
}

// Right arm servo
uint8_t servo_right_arm_get_angle(void)
{
  return ausbeeGetAngleServo(&servo_right_arm);
}

void servo_right_arm_set_angle(uint8_t angle)
{
  ausbeeSetAngleServo(&servo_right_arm, angle);
}

void servo_right_arm_open(void)
{
  servo_right_arm_set_angle(15);
}

void servo_right_arm_close(void)
{
  servo_right_arm_set_angle(75);
}
