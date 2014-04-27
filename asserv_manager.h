#ifndef ASSERV_MANAGER_H
#define ASSERV_MANAGER_H

#include <AUSBEE/control_system_manager.h>
#include <AUSBEE/pid.h>
#include "trajectory_manager.h"

struct asserv_manager {
  struct ausbee_cs csm_right_motor;
  struct ausbee_cs csm_left_motor;

  struct ausbee_pid pid_right_motor;
  struct ausbee_pid pid_left_motor;
};

void start_control_system(struct asserv_manager *, struct trajectory_manager *);

#endif /* ASSERV_MANAGER_H */
