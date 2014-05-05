#ifndef POSITION_MANAGER_H
#define POSITION_MANAGER_H

#include <stdint.h>

void position_init(void);
void position_update(int32_t left_enc_diff, int32_t right_enc_diff);

void position_set_tick_per_meter(uint32_t tick_per_cm);
void position_set_axle_track_mm(uint32_t d);

int32_t position_get_left_encoder(void);
int32_t position_get_right_encoder(void);
double  position_get_distance_mm(void);
double  position_get_enc_diff_mm(void);
double  position_get_enc_diff_by_axle_track(void);
double  position_get_angle_diff_rad(void);
double  position_get_angle_diff_deg(void);
double  position_get_angle_deg(void);

#endif /* POSITION_MANAGER_H */
