#include "position_manager.h"
#include <math.h>

#define PI 3.1415926535L

struct position_manager {
  uint32_t tick_per_m;
  uint32_t axle_track_mm;

  int32_t left_encoder, right_encoder;
  int32_t distance_mm;
  int32_t angle_deg;
} pm;

int32_t position_ticks_to_mm(uint32_t value_ticks);

void position_init(void)
{
  pm.tick_per_m = 0;

  pm.left_encoder = 0;
  pm.right_encoder = 0;

  pm.distance_mm = 0;
  pm.angle_deg = 0;
}

void position_update(int32_t left_enc_diff, int32_t right_enc_diff)
{
  pm.left_encoder += left_enc_diff;
  pm.right_encoder += right_enc_diff;

  int32_t distance_ticks = (pm.left_encoder + pm.right_encoder) / 2;
  pm.distance_mm = position_ticks_to_mm(distance_ticks);

  int32_t wheel_distance_diff_mm = position_ticks_to_mm(pm.left_encoder - pm.right_encoder);
  pm.angle_deg = atan2(wheel_distance_diff_mm, pm.axle_track_mm) * 180 / PI;

  // TODO: compute x and y
}

int32_t position_ticks_to_mm(uint32_t value_ticks)
{
  return value_ticks * 1000 / pm.tick_per_m;
}

void position_set_tick_per_meter(uint32_t tick_per_m)
{
  pm.tick_per_m = tick_per_m;
}

void position_set_axle_track_mm(uint32_t d)
{
  pm.axle_track_mm = d;
}

int32_t position_get_left_encoder(void)
{
  return pm.left_encoder;
}

int32_t position_get_right_encoder(void)
{
  return pm.right_encoder;
}

int32_t position_get_distance_mm(void)
{
  return pm.distance_mm;
}

int32_t position_get_angle_deg(void)
{
  return pm.angle_deg;
}
