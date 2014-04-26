#ifndef ASSERV_MANAGER_H
#define ASSERV_MANAGER_H

#include <AUSBEE/control_system_manager.h>
#include "trajectory_manager.h"

void start_control_system(struct ausbee_cs *, struct trajectory_manager *);

#endif /* ASSERV_MANAGER_H */
