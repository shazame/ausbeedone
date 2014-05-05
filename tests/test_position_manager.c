#include "../utils/position_manager.h"
#include <inttypes.h>
#include <stdio.h>

int main(void)
{
  position_init();
  position_set_tick_per_meter(1170);
  position_set_axle_track_mm(235);
  int i;

  for (i = 0; i < 10; i++) {
    position_update(0, 19);

    printf("Robot Distance mm:            %lf\n"  , position_get_distance_mm());
    printf("Robot Enc diff mm:            %lf\n"  , position_get_enc_diff_mm());
    printf("Robot Enc diff by axle track: %lf\n"  , position_get_enc_diff_by_axle_track());
    printf("Robot Angle diff rad:         %lf\n"  , position_get_angle_diff_rad());
    printf("Robot Angle diff deg:         %lf\n"  , position_get_angle_diff_deg());
    printf("Robot Angle:                  %lf\n\n", position_get_angle_deg());
  }

  return 0;
}
