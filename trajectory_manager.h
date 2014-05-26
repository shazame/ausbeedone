#ifndef TRAJECTORY_MANAGER_H
#define TRAJECTORY_MANAGER_H

#include "control_system.h"

#define TRAJECTORY_UPDATE_PERIOD_S 0.1 // 100 ms

#define TRAJECTORY_MAX_NB_POINTS 50

#define TRAJECTORY_DEFAULT_PRECISION_D_MM  10.0
#define TRAJECTORY_DEFAULT_PRECISION_A_DEG 1.0

enum trajectory_order_type {
  PAUSE, D, A_ABS, A_REL
};

enum trajectory_when {
  NOW, END
};

struct trajectory_dest {
  union {

    struct {
      float mm;
      float precision;
    } d;

    struct {
      float deg;
      float precision;
    } a;

  };

  float starting_d_mm;
  float starting_a_deg;
  uint8_t is_init;
  enum trajectory_order_type type;
};

struct trajectory_manager {
  struct trajectory_dest points[TRAJECTORY_MAX_NB_POINTS];
  uint32_t cur_id;
  uint32_t last_id;

  struct control_system *cs;
};

void trajectory_init(struct trajectory_manager *t,
                     struct control_system     *cs);

void trajectory_start(struct trajectory_manager *t);

/* Remove every points from the trajectory */
void trajectory_end(struct trajectory_manager *t);
/* Check whether points remain in the trajectory*/
int trajectory_is_ended(struct trajectory_manager *t);

void trajectory_next_point(struct trajectory_manager *t);

uint32_t trajectory_get_cur_id(struct trajectory_manager *t);
uint32_t trajectory_get_last_id(struct trajectory_manager *t);

void trajectory_pause(struct trajectory_manager *t);
void trajectory_resume(struct trajectory_manager *t);

void trajectory_goto_d_mm(struct trajectory_manager *t, float d_mm);

/* Set absolute angle. Does not depend on current angle. */
void trajectory_goto_a_abs_deg(struct trajectory_manager *t,
                               float a_deg_ref);

/* Set relative angle. Depends on current angle. */
void trajectory_goto_a_rel_deg(struct trajectory_manager *t,
                               float a_deg);
#endif /* TRAJECTORY_MANAGER_H */
