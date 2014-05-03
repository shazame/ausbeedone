#ifndef POSITION_MANAGER_H
#define POSITION_MANAGER_H

#include <stdint.h>

void position_init(void);
void position_update(int32_t left_enc, int32_t right_enc);

void position_set_tick_per_meter(uint32_t tick_per_cm);

int32_t position_get_left_encoder(void);
int32_t position_get_right_encoder(void);
int32_t position_get_distance(void);
int32_t position_get_angle(void);

#endif /* POSITION_MANAGER_H */
