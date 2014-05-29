#ifndef LIDAR_DETECT_H
#define LIDAR_DETECT_H

#include "../trajectory_manager.h"

void lidar_detect_start(struct trajectory_manager *t);
uint8_t lidar_detect_obstacle(void);

void lidar_detect_enable(void);
void lidar_detect_disable(void);

#endif /* LIDAR_DETECT_H */
