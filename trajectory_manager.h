#ifndef TRAJECTORY_MANAGER_H
#define TRAJECTORY_MANAGER_H

#include "control_system.h"

struct trajectory_manager {
  struct control_system *cs;
};

void trajectory_init(struct trajectory_manager *t,
                     struct control_system     *cs);

void trajectory_start(struct trajectory_manager *t);

void trajectory_goto_d_mm(struct trajectory_manager *t, float d_mm);

/* Set absolute angle. Does not depend on current angle. */
void trajectory_goto_a_abs_deg(struct trajectory_manager *t,
                               float a_deg_ref);

/* Set relative angle. Depends on current angle. */
void trajectory_goto_a_rel_deg(struct trajectory_manager *t,
                               float a_deg);

#endif /* TRAJECTORY_MANAGER_H */
