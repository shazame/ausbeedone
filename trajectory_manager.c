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
                                 struct trajectory_dest point,
                                 enum trajectory_when when);

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
  xTaskCreate(trajectory_task, (const signed char *)"TrajectoryManager", 200, (void *)t, 1, NULL );
}

void trajectory_end(struct trajectory_manager *t)
{
  t->cur_id = t->last_id;
}

int trajectory_is_ended(struct trajectory_manager *t)
{
  return (t->cur_id == t->last_id);
}

void trajectory_next_point(struct trajectory_manager *t)
{
  /* Update list pointer if not empty */
  if (!trajectory_is_ended(t)) {
    t->cur_id = (t->cur_id+1) % TRAJECTORY_MAX_NB_POINTS;
  }
}

uint32_t trajectory_get_cur_id(struct trajectory_manager *t)
{
  return t->cur_id;
}

uint32_t trajectory_get_last_id(struct trajectory_manager *t)
{
  return t->last_id;
}

/******************** Movement functions ********************/

static int trajectory_is_paused(struct trajectory_manager *t)
{
  return (!trajectory_is_ended(t) &&
          (t->points[t->cur_id].type == PAUSE));
}

void trajectory_pause(struct trajectory_manager *t)
{
  if (!trajectory_is_paused(t)) {
      struct trajectory_dest dest;
      dest.type = PAUSE;
      trajectory_add_point(t, dest, NOW);
  }

  /* Force update now to stop more quickly */
  trajectory_update(t);
}

void trajectory_resume(struct trajectory_manager *t)
{
  while (trajectory_is_paused(t)) {
    trajectory_next_point(t);
  }
}

void trajectory_goto_d_mm(struct trajectory_manager *t, float d_mm)
{
  struct trajectory_dest dest;

  dest.type = D;
  dest.d.mm = d_mm;
  dest.d.precision = TRAJECTORY_DEFAULT_PRECISION_D_MM;

  trajectory_add_point(t, dest, END);
}

/* Absolute angle = current angle + relative angle. */
void trajectory_goto_a_abs_deg(struct trajectory_manager *t,
                               float a_deg)
{
  struct trajectory_dest dest;

  dest.type = A_ABS;
  dest.a.deg = a_deg;
  dest.a.precision = TRAJECTORY_DEFAULT_PRECISION_A_DEG;

  trajectory_add_point(t, dest, END);
}

void trajectory_goto_a_rel_deg(struct trajectory_manager *t,
                               float a_deg)
{
  struct trajectory_dest dest;

  dest.type = A_REL;
  dest.a.deg = a_deg;
  dest.a.precision = TRAJECTORY_DEFAULT_PRECISION_A_DEG;

  trajectory_add_point(t, dest, END);
}

/****************** Internal functions ******************/

void trajectory_task(void *data)
{
  struct trajectory_manager *t = (struct trajectory_manager *)data;

  for (;;) {
    trajectory_update(t);
    vTaskDelay(TRAJECTORY_UPDATE_PERIOD_S * 1000 / portTICK_RATE_MS);
  }
}

static inline int trajectory_is_full(struct trajectory_manager *t)
{
  return (((t->last_id+1) % TRAJECTORY_MAX_NB_POINTS) == t->cur_id);
}

static inline void trajectory_decrease_id(uint32_t *id)
{
  *id = (*id + TRAJECTORY_MAX_NB_POINTS - 1) % TRAJECTORY_MAX_NB_POINTS;
}

void trajectory_add_point(struct trajectory_manager *t,
                          struct trajectory_dest point,
                          enum trajectory_when when)
{
  point.is_init = 0;

  if (when == END) {
    if (trajectory_is_full(t)) {
      printf("[trajectory_manager] Warning: List of points is full. Last point not added.\n");
      return;
    }

    /* New points are added at the end of the list */
    t->points[t->last_id] = point;

    /* Update end of list pointer */
    t->last_id = (t->last_id + 1) % TRAJECTORY_MAX_NB_POINTS;
  }
  else if (when == NOW) {
    if (trajectory_is_full(t)) {
      trajectory_decrease_id(&(t->last_id));
      printf("[trajectory_manager] Warning: List of points is full. Last point was removed.\n");
    }

    /* Insert a point before the current one */
    trajectory_decrease_id(&(t->cur_id));
    t->points[t->cur_id] = point;
  }
}

static void trajectory_manage_order_d(struct trajectory_manager *t,
                               struct trajectory_dest *p)
{
  float d_mm_ref = p->starting_d_mm + p->d.mm;
  if (ABS(d_mm_ref - position_get_distance_mm(NULL)) < p->d.precision) {
    trajectory_next_point(t);
  }
  else {
    control_system_set_distance_mm_ref(t->cs, d_mm_ref);
  }
}

static void trajectory_manage_order_a_abs(struct trajectory_manager *t,
                                   struct trajectory_dest *p)
{
  // TODO: Better handle
  if (ABS(p->a.deg - position_get_angle_deg(NULL)) < p->a.precision) {
    trajectory_next_point(t);
  }
  else {
    control_system_set_angle_deg_ref(t->cs, p->a.deg);
  }
}

static void trajectory_manage_order_a_rel(struct trajectory_manager *t,
                                   struct trajectory_dest *p)
{
  float a_deg_ref = p->starting_a_deg + p->a.deg;
  if (ABS(a_deg_ref - position_get_angle_deg(NULL)) < p->a.precision) {
    trajectory_next_point(t);
  }
  else {
    control_system_set_angle_deg_ref(t->cs, a_deg_ref);
  }
}

static void trajectory_manage_order_pause(struct trajectory_manager *t,
                                          struct trajectory_dest *p)
{
  control_system_set_distance_mm_ref(t->cs, p->starting_d_mm);
  control_system_set_angle_deg_ref(t->cs, p->starting_a_deg);
}

inline void trajectory_update(struct trajectory_manager *t)
{
  /* Nothing to do if there is no point in the list */
  if (t->cur_id == t->last_id) {
    return;
  }

  /* Get current point reference */
  struct trajectory_dest *p = t->points + t->cur_id;

  /* Get current starting position in distance and angle */
  if (!p->is_init) {
    p->starting_d_mm = position_get_distance_mm(NULL);
    p->starting_a_deg = position_get_angle_deg(NULL);
    p->is_init = 1;
  }

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
    case PAUSE:
      trajectory_manage_order_pause(t, p);
      break;
    default:
      break;
  }
}
