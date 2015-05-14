#ifndef GP2_DETECT_H
#define GP2_DETECT_H

#include "../trajectory_manager.h"

void gp2_detect_start(struct trajectory_manager *t, struct control_system *am);
uint8_t gp2_detect_obstacle(void);

void gp2_detect_enable(void);
void gp2_detect_disable(void);

#endif /* GP2_DETECT_H */
