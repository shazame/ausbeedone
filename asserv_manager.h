#ifndef ASSERV_MANAGER_H
#define ASSERV_MANAGER_H

#include <AUSBEE/control_system_manager.h>
#include <AUSBEE/pid.h>

#include "utils/motors_wrapper.h"

struct asserv_manager {
  struct ausbee_cs csm_right_motor;
  struct ausbee_cs csm_left_motor;

  struct ausbee_pid pid_right_motor;
  struct ausbee_pid pid_left_motor;
};

void start_control_system(struct asserv_manager *);

void control_system_set_right_motor_ref(struct asserv_manager *am, int32_t ref);
void control_system_set_left_motor_ref(struct asserv_manager *am, int32_t ref);

#endif /* ASSERV_MANAGER_H */
