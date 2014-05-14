/**
 * @file    position_manager.c
 * @author  David BITONNEAU <david.bitonneau@gmail.com>
 * @version V1.0
 * @date    12-Mar-2014
 * @brief   An odometry module. Implementation file.
 */

#include "position_manager.h"
#include <math.h>
#include <stdio.h>

#ifdef UNUSED
#elif defined(__GNUC__)
# define UNUSED(x) UNUSED_ ## x __attribute__((unused))
#elif defined(__LCLINT__)
# define UNUSED(x) /*@unused@*/ x
#else
# define UNUSED(x) x
#endif

#define PI 3.1415926535

struct position_manager {
  uint32_t ticks_per_m;
  uint32_t axle_track_mm;

  int32_t left_encoder, right_encoder;

  float distance_mm;
  float angle_rad;
  float x_mm, y_mm;
} pm;

#define position_ticks_to_mm(value_ticks) ((value_ticks) * 1000.0 / pm.ticks_per_m)

void position_init(void)
{
  pm.ticks_per_m = 0;

  pm.left_encoder = 0;
  pm.right_encoder = 0;

  pm.distance_mm = 0;
  pm.angle_rad = 0;

  pm.x_mm = 0;
  pm.y_mm = 0;
}

void position_update(int32_t left_enc_diff, int32_t right_enc_diff)
{
  pm.left_encoder += left_enc_diff;
  pm.right_encoder += right_enc_diff;

  // Distance
  float distance_diff_ticks = (left_enc_diff + right_enc_diff) / 2.0;
  float distance_diff_mm = position_ticks_to_mm(distance_diff_ticks);
  pm.distance_mm += distance_diff_mm;

  // Special case: no rotation
  if ((right_enc_diff - left_enc_diff) == 0)
  {
    pm.x_mm += distance_diff_mm * (float)sin(pm.angle_rad);
    pm.y_mm += distance_diff_mm * (float)cos(pm.angle_rad);
    return;
  }

  // Angle
  float angle_diff_rad = position_ticks_to_mm(right_enc_diff - left_enc_diff) / pm.axle_track_mm;

  // Special case: only rotation -> no need to update x and y
  if ((right_enc_diff + left_enc_diff) == 0)
  {
    pm.angle_rad += angle_diff_rad;
    return;
  }

  // Radius of curvature
  float r = pm.axle_track_mm / 2.0 * (right_enc_diff + left_enc_diff) /
                                     (right_enc_diff - left_enc_diff);

  // Trajectory circle center coordinates
  float x0_mm = pm.x_mm - r * (float)cos(pm.angle_rad);
  float y0_mm = pm.y_mm - r * (float)sin(pm.angle_rad);

  // Update position
  pm.angle_rad += angle_diff_rad;
  pm.x_mm       = x0_mm + r * (float)cos(pm.angle_rad);
  pm.y_mm       = y0_mm + r * (float)sin(pm.angle_rad);
}

void position_set_tick_per_meter(uint32_t ticks_per_m)
{
  pm.ticks_per_m = ticks_per_m;
}

void position_set_axle_track_mm(uint32_t d)
{
  pm.axle_track_mm = d;
}

uint32_t position_get_axle_track_mm(void)
{
  return pm.axle_track_mm;
}

float position_get_left_encoder(void *UNUSED(dummy))
{
  return pm.left_encoder;
}

float position_get_right_encoder(void *UNUSED(dummy))
{
  return pm.right_encoder;
}

float position_get_distance_mm(void)
{
  return pm.distance_mm;
}

float position_get_angle_deg(void)
{
  return pm.angle_rad * 180.0 / PI;
}

float position_get_x_mm(void)
{
  return pm.x_mm;
}

float position_get_y_mm(void)
{
  return pm.y_mm;
}

int32_t position_mm_to_ticks(float value_mm)
{
  return value_mm * pm.ticks_per_m / 1000.0;
}
