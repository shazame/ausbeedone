#include "position_manager.h"

struct position_manager {
  uint32_t tick_per_m; // Number of ticks per cm

  int32_t left_encoder, right_encoder;
  int32_t distance; // In cm
  int32_t angle;
} pm;

void position_init(void)
{
  pm.tick_per_m = 0;

  pm.left_encoder = 0;
  pm.right_encoder = 0;

  pm.distance = 0;
  pm.angle = 0;
}

void position_update(int32_t left_enc, int32_t right_enc)
{
  pm.left_encoder = left_enc;
  pm.right_encoder = right_enc;

  pm.distance = (pm.left_encoder + pm.right_encoder) * 100 / (2 * pm.tick_per_m);

  // TODO: update angle and other things
}

void position_set_tick_per_meter(uint32_t tick_per_m)
{
  pm.tick_per_m = tick_per_m;
}

int32_t position_get_left_encoder(void)
{
  return pm.left_encoder;
}

int32_t position_get_right_encoder(void)
{
  return pm.right_encoder;
}

int32_t position_get_distance(void)
{
  return pm.distance;
}

int32_t position_get_angle(void)
{
  return pm.angle;
}
