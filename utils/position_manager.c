#include "position_manager.h"
#include <math.h>

#define PI 3.1415926535L

struct position_manager {
  uint32_t tick_per_m;
  uint32_t axle_track_mm;

  int32_t left_encoder, right_encoder;
  double  distance_mm;
  double  enc_diff_mm;
  double  enc_diff_by_axle_track;
  double  angle_diff_rad;
  double  angle_diff_deg;
  double  angle_deg;
} pm;

#define position_ticks_to_mm(value_ticks) ((value_ticks) * 1000.0L / pm.tick_per_m)

void position_init(void)
{
  pm.tick_per_m = 0;

  pm.left_encoder = 0;
  pm.right_encoder = 0;

  pm.distance_mm = 0;
  pm.enc_diff_mm = 0;
  pm.enc_diff_by_axle_track = 0;
  pm.angle_diff_rad = 0;
  pm.angle_diff_deg = 0;
  pm.angle_deg = 0;
}

void position_update(int32_t left_enc_diff, int32_t right_enc_diff)
{
  pm.left_encoder += left_enc_diff;
  pm.right_encoder += right_enc_diff;

  double distance_ticks = (pm.left_encoder + pm.right_encoder) / 2.0;
  pm.distance_mm = position_ticks_to_mm(distance_ticks);

  pm.enc_diff_mm = position_ticks_to_mm(left_enc_diff - right_enc_diff);
  pm.enc_diff_by_axle_track = 1.0L * pm.enc_diff_mm / pm.axle_track_mm;
  pm.angle_diff_rad = atan(pm.enc_diff_by_axle_track);
  pm.angle_diff_deg = pm.angle_diff_rad * 180.0L / PI;
  pm.angle_deg += pm.angle_diff_deg;

  // TODO: compute x and y
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

double position_get_distance_mm(void)
{
  return pm.distance_mm;
}

double position_get_enc_diff_mm(void)
{
  return pm.enc_diff_mm;
}

double position_get_enc_diff_by_axle_track(void)
{
  return pm.enc_diff_by_axle_track;
}

double position_get_angle_diff_rad(void)
{
  return pm.angle_diff_rad;
}

double position_get_angle_diff_deg(void)
{
  return pm.angle_diff_deg;
}

double position_get_angle_deg(void)
{
  return pm.angle_deg;
}
