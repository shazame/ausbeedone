#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "utils/position_manager.h"
#include "cli.h"

void cli_task(void *);

void cli_start(struct trajectory_manager *t)
{
  xTaskCreate(cli_task, (const signed char *)"CLI", 200, (void *)t, 1, NULL );
}

static int cli_getchar(void)
{
  int c = getchar();

  if (c == CLI_END_CHAR) {
    printf("\r\n");
  }
  else {
    printf("%c", (char)c);
  }

  return c;
}

static void cli_getstr(char *buff, uint8_t str_length)
{
  int c = 0;
  int i = 0;

  memset(buff, 0, str_length * sizeof(char));

  while (((c = cli_getchar()) != EOF) && ((char)c == CLI_DELIMITER));

  do {
    buff[i++] = (char)c;

    if (i > str_length)
      break;

  } while (((c = cli_getchar()) != EOF) && ((char)c != CLI_DELIMITER) && ((char) c != CLI_END_CHAR));
}

#define FLOAT_LENGTH 20
static float cli_getfloat(void)
{
  char buff[FLOAT_LENGTH] = {0};

  cli_getstr(buff, FLOAT_LENGTH);

  float value = strtof(buff, NULL);

  return value;
}

#define ARG_LENGTH 20
void cli_task(void *data)
{
  struct trajectory_manager *t = (struct trajectory_manager *)data;

  char command = 0;
  float value = 0;
  char arg[ARG_LENGTH] = {0};
  int c = 0;

  for (;;) {
    value = 0;

    printf("$ ");
    command = cli_getchar();
    if (command == 'd' || command == 'a') {
      value = cli_getfloat();
    }
    else if (command == 'p') {
      cli_getstr(arg, ARG_LENGTH);
    }
    else if (command == 's') {
      cli_getstr(arg, ARG_LENGTH);
      value = cli_getfloat();
    }
    else {
      while (((c = cli_getchar()) != EOF) && ((char)c != CLI_END_CHAR));
    }

    if (command == 'd') {
      trajectory_goto_d_mm(t, value);
      printf("Distance: %f\r\n", (double)value);
    }
    else if (command == 'a') {
      trajectory_goto_a_rel_deg(t, value);
      printf("Angle: %f\r\n", (double)value);
    }
    else if (command == 's') {
      if (!strncmp(arg, "d_max_speed", ARG_LENGTH)) {
        control_system_set_distance_max_speed(t->cs, value);
        printf("Max translation speed: %f\r\n", (double)value);
      }
      else if (!strncmp(arg, "d_max_acc", ARG_LENGTH)) {
        control_system_set_distance_max_acc(t->cs, value);
        printf("Max translation acceleration: %f\r\n", (double)value);
      }
      else if (!strncmp(arg, "a_max_speed", ARG_LENGTH)) {
        control_system_set_angle_max_speed(t->cs, value);
        printf("Max rotation speed: %f\r\n", (double)value);
      }
      else if (!strncmp(arg, "a_max_acc", ARG_LENGTH)) {
        control_system_set_angle_max_acc(t->cs, value);
        printf("Max rotation acceleration: %f\r\n", (double)value);
      }
      else {
        printf("Invalid argument '%s'.\r\n", arg);
      }
    }
    else if (command == 'p') {
      if (!strncmp(arg, "x", ARG_LENGTH)) {
        printf("Robot x mm: %f\r\n", (double)position_get_x_mm());
      }
      else if (!strncmp(arg, "y", ARG_LENGTH)) {
        printf("Robot y mm: %f\r\n", (double)position_get_y_mm());
      }
      else if (!strncmp(arg, "a", ARG_LENGTH)) {
        printf("Robot angle deg: %f\r\n", (double)position_get_angle_deg(NULL));
      }
      else if (!strncmp(arg, "d", ARG_LENGTH)) {
        printf("Robot distance mm: %f\r\n", (double)position_get_distance_mm(NULL));
      }
      else if (!strncmp(arg, "enc_r", ARG_LENGTH)) {
        printf("Right encoder value: %f\r\n", (double)position_get_right_encoder(NULL));
      }
      else if (!strncmp(arg, "enc_l", ARG_LENGTH)) {
        printf("Left encoder value: %f\r\n", (double)position_get_left_encoder(NULL));
      }
      else if (!strncmp(arg, "cur_id", ARG_LENGTH)) {
        printf("Position manager cur_id: %"PRId32"\r\n", trajectory_get_cur_id(t));
      }
      else if (!strncmp(arg, "last_id", ARG_LENGTH)) {
        printf("Position manager last_id: %"PRId32"\r\n", trajectory_get_last_id(t));
      }
      else {
        printf("Invalid argument '%s'.\r\n", arg);
      }
    }
    else if (command == 'h') {
      printf("Help:\r\n");
      printf("  Available commands are:\r\n");
      printf("  d <float>: Go forward/backward with the specified distance in mm.\r\n");
      printf("  a <float>: Rotate with the specified angle in degrees.\r\n");
      printf("  s <arg> <value>:   Set internal value.\r\n");
      printf("             <value> should be a float.\r\n");
      printf("             <arg> can be one of:\r\n");
      printf("             d_max_speed: set max translation speed.\r\n");
      printf("             d_max_acc:   set max translation acceleration.\r\n");
      printf("             a_max_speed: set max rotation speed.\r\n");
      printf("             a_max_acc:   set max rotation acceleration.\r\n");
      printf("  p <arg>:   Print internal value.\r\n");
      printf("             <arg> can be one of:\r\n");
      printf("             x:        print robot's x position.\r\n");
      printf("             y:        print robot's y position.\r\n");
      printf("             a:        print robot's angle.\r\n");
      printf("             d:        print robot's distance.\r\n");
      printf("             enc_l:    print left encoder's value.\r\n");
      printf("             enc_r:    print right encoder's value.\r\n");
      printf("             cur_id:   print position manager's current point id.\r\n");
      printf("             last_id:  print position manager's last point id.\r\n");
      printf("  h: Display this help.\r\n");
    }
    else {
      printf("Unknown command '%c'. Type 'h' for help.\r\n", command);
    }

    vTaskDelay(1 / portTICK_RATE_MS);
  }
}
