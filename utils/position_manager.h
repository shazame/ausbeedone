/**
 * @file    position_manager.h
 * @author  David BITONNEAU <david.bitonneau@gmail.com>
 * @version V1.0
 * @date    12-Mar-2014
 * @brief   An odometry module. Definition file.
 */

#ifndef POSITION_MANAGER_H
#define POSITION_MANAGER_H

#include <stdint.h>

void position_init(void);
void position_update(int32_t left_enc_diff, int32_t right_enc_diff);

void position_set_tick_per_meter(uint32_t tick_per_m);
void position_set_axle_track_mm(uint32_t d);

uint32_t position_get_axle_track_mm(void);

float position_get_left_encoder(void *dummy);
float position_get_right_encoder(void *dummy);

float position_get_distance_mm(void *dummy);
float position_get_angle_rad(void *dummy);
float position_get_angle_deg(void *dummy);
float position_get_x_mm(void);
float position_get_y_mm(void);

int32_t position_mm_to_ticks(float value_mm);

#endif /* POSITION_MANAGER_H */
