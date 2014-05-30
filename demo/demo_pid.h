#ifndef DEMO_PID_H
#define DEMO_PID_H

#include "../trajectory_manager.h"

void demo_pid_start(struct trajectory_manager *t, float angle_coeff);
void demo_pid_stop(void);

#endif /* DEMO_PID_H */
