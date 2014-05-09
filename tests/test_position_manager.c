#include "../utils/position_manager.h"
#include <inttypes.h>
#include <stdio.h>

#define NB_POINTS 4
int32_t traj[NB_POINTS][2] = {
  {20, 20},
  {216, -216},
  {40, 40},
  {0, 432}
};

int main(void)
{
  position_init();
  position_set_tick_per_meter(1170);
  position_set_axle_track_mm(235);
  int i;

  for (i = 0; i < NB_POINTS; i++) {
    position_update(traj[i][0], traj[i][1]);

    printf("Robot Distance mm: %f\n"  , position_get_distance_mm());
    printf("Robot Angle:       %f\n", position_get_angle_deg());
    printf("Robot x mm:        %f\n"  , position_get_x_mm());
    printf("Robot y mm:        %f\n\n", position_get_y_mm());
  }

  return 0;
}
