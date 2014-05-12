/**
 * @file    asserv_manager.h
 * @author  David BITONNEAU <david.bitonneau@gmail.com>
 * @version V1.0
 * @date    11-Mar-2014
 * @brief   A controller system for a two-wheeled with encoders.
 *          Definition file.
 */

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

void control_system_start(struct asserv_manager *);

void control_system_set_right_motor_ref(struct asserv_manager *am, int32_t ref);
void control_system_set_left_motor_ref(struct asserv_manager *am, int32_t ref);

#endif /* ASSERV_MANAGER_H */
