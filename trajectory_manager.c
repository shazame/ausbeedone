#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "utils/position_manager.h"
#include "trajectory_manager.h"

#define ABS(x) (((x) < 0)? -(x): (x))

/********************   Prototypes   ********************/

void trajectory_task(void *data);
static inline void trajectory_update(struct trajectory_manager *t);
static void trajectory_add_point(struct trajectory_manager *t,
                                 struct trajectory_dest point);

/******************** User functions ********************/

void trajectory_init(struct trajectory_manager *t,
                     struct control_system *cs)
{
  t->cs = cs;

  t->cur_id = 0;
  t->last_id = 0;
}

void trajectory_start(struct trajectory_manager *t)
{
  xTaskCreate(trajectory_task, (const signed char *)"TrajectoryManager", 100, (void *)t, 1, NULL );
}

uint32_t trajectory_get_cur_id(struct trajectory_manager *t)
{
  return t->cur_id;
}

uint32_t trajectory_get_last_id(struct trajectory_manager *t)
{
  return t->last_id;
}

/******************** User functions ********************/

void trajectory_goto_d_mm(struct trajectory_manager *t, float d_mm)
{
  struct trajectory_dest dest;
  float d_mm_ref = position_get_distance_mm(NULL) + d_mm;

  dest.type = D;
  dest.d.mm = d_mm_ref;
  dest.d.precision = TRAJECTORY_DEFAULT_PRECISION_D_MM;

  trajectory_add_point(t, dest);
}

/* Absolute angle = current angle + relative angle. */
void trajectory_goto_a_abs_deg(struct trajectory_manager *t,
                               float a_deg_ref)
{
  struct trajectory_dest dest;

  dest.type = A_ABS;
  dest.a_abs.deg = a_deg_ref;
  dest.a_abs.precision = TRAJECTORY_DEFAULT_PRECISION_A_DEG;

  trajectory_add_point(t, dest);
}

void trajectory_goto_a_rel_deg(struct trajectory_manager *t,
                               float a_deg)
{
  struct trajectory_dest dest;
  float a_deg_ref = position_get_angle_deg(NULL) + a_deg;

  dest.type = A_REL;
  dest.a_rel.deg = a_deg_ref;
  dest.a_rel.precision = TRAJECTORY_DEFAULT_PRECISION_A_DEG;

  trajectory_add_point(t, dest);
}

/****************** Internal functions ******************/

void trajectory_task(void *data)
{
  struct trajectory_manager *t = (struct trajectory_manager *)data;

  for (;;) {
    trajectory_update(t);
    vTaskDelay(100 / portTICK_RATE_MS);
  }
}

static void trajectory_next_point(struct trajectory_manager *t)
{
  /* Update list pointer if not empty */
  if (t->cur_id != t->last_id) {
    t->cur_id = (t->cur_id+1) % TRAJECTORY_MAX_NB_POINTS;
  }
}

void trajectory_add_point(struct trajectory_manager *t,
                          struct trajectory_dest point)
{
  /* If the list is full, the new point is not added */
  if (((t->last_id+1) % TRAJECTORY_MAX_NB_POINTS) == t->cur_id) {
    printf("[trajectory_manager] Warning: List of points is full. Last point not added.\n");
    return;
  }

  /* New points are added at the end of the list */
  t->points[t->last_id] = point;

  /* Update end of list pointer */
  t->last_id = (t->last_id +1 ) % TRAJECTORY_MAX_NB_POINTS;
}

void trajectory_manage_order_d(struct trajectory_manager *t,
                               struct trajectory_dest *p)
{
  if (ABS(p->d.mm - position_get_distance_mm(NULL)) < p->d.precision) {
    trajectory_next_point(t);
  }
  else {
    control_system_set_distance_mm_ref(t->cs, p->d.mm);
  }
}

void trajectory_manage_order_a_abs(struct trajectory_manager *t,
                                   struct trajectory_dest *p)
{
  // TODO: Better handle
  if (ABS(p->a_abs.deg - position_get_angle_deg(NULL)) < p->a_abs.precision) {
    trajectory_next_point(t);
  }
  else {
    control_system_set_angle_deg_ref(t->cs, p->a_abs.deg);
  }
}

void trajectory_manage_order_a_rel(struct trajectory_manager *t,
                                   struct trajectory_dest *p)
{
  if (ABS(p->a_rel.deg - position_get_angle_deg(NULL)) < p->a_rel.precision) {
    trajectory_next_point(t);
  }
  else {
    control_system_set_angle_deg_ref(t->cs, p->a_rel.deg);
  }
}

inline void trajectory_update(struct trajectory_manager *t)
{
  /* Nothing to do if there is no point in the list */
  if (t->cur_id == t->last_id) {
    return;
  }

  /* Get current point reference */
  struct trajectory_dest *p = t->points + t->cur_id;

  /* Set new reference according to point type */
  switch (p->type) {
    case D:
      trajectory_manage_order_d(t, p);
      break;
    case A_ABS:
      trajectory_manage_order_a_abs(t, p);
      break;
    case A_REL:
      trajectory_manage_order_a_rel(t, p);
      break;
    default:
      break;
  }
}
