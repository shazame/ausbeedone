#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

void servo_init_starting_position(void);

uint8_t servo_left_arm_get_angle(void);
void servo_left_arm_set_angle(uint8_t angle);
void servo_left_arm_open(void);
void servo_left_arm_close(void);

uint8_t servo_right_arm_get_angle(void);
void servo_right_arm_set_angle(uint8_t angle);
void servo_right_arm_open(void);
void servo_right_arm_close(void);

#endif /* end of include guard: SERVO_H */
