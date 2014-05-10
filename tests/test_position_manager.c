#include "../utils/position_manager.h"
#include <inttypes.h>
#include <stdio.h>

#define NB_POINTS 40
int32_t traj[NB_POINTS][2] = {
  {40, 40},
  {40, 40},
  {40, 40},
  {40, 40},
  {40, 40},
  {41, -41},
  {41, -41},
  {41, -41},
  {41, -41},
  {41, -41},
  {41, -41},
  {41, -41},
  {41, -41},
  {41, -41},
  {41, -41},
  {50, 50},
  {50, 50},
  {50, 50},
  {50, 50},
  {50, 50},
  {0, 83},
  {0, 83},
  {0, 83},
  {0, 83},
  {0, 83},
  {0, 43},
  {0, 43},
  {0, 43},
  {0, 43},
  {0, 43},
  {80, 30},
  {80, 30},
  {80, 30},
  {80, 30},
  {80, 30},
  {50, 20},
  {50, 20},
  {50, 20},
  {50, 20},
  {50, 20}
};

int main(void)
{
  position_init();
  position_set_tick_per_meter(1170);
  position_set_axle_track_mm(235);
  int i;

  for (i = 0; i < NB_POINTS; i++) {
    position_update(traj[i][0], traj[i][1]);

    printf("Right: 0: 1\n");
    printf("Left:  0: 1\n");

    printf("Odometry Distance mm: %f: 1;"   , position_get_distance_mm());
    printf("Odometry Angle:       %f: 1\r\n", position_get_angle_deg());

    printf("Robot x mm: %f;"   , position_get_x_mm());
    printf("Robot y mm: %f\r\n", position_get_y_mm());
  }

  return 0;
}
