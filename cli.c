#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "utils/position_manager.h"
#include "cli.h"

void cli_task(void *);

void cli_start(struct control_system *am)
{
  xTaskCreate(cli_task, (const signed char *)"CLI", 240, (void *)am, 1, NULL );
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

  if ((char) c != CLI_END_CHAR) {
    while (((c = cli_getchar()) != EOF) && ((char)c != CLI_END_CHAR));
  }
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
  struct control_system *am = (struct control_system *)data;

  char command = 0;
  float value = 0;
  char arg[ARG_LENGTH] = {0};
  int c = 0;

  for(;;) {
    value = 0;

    printf("$ ");
    command = cli_getchar();
    if (command == 'd' || command == 'a') {
      value = cli_getfloat();
    }
    else if (command == 'p') {
      cli_getstr(arg, ARG_LENGTH);
    }
    else {
      while (((c = cli_getchar()) != EOF) && ((char)c != CLI_END_CHAR));
    }

    if (command == 'd') {
      control_system_set_distance_mm_ref(am, value);
      printf("Distance: %f\r\n", (double)value);
    }
    else if (command == 'a') {
      control_system_set_angle_deg_ref(am, value);
      printf("Angle: %f\r\n", (double)value);
    }
    else if (command == 'p') {
      if (!strncmp(arg, "x", ARG_LENGTH)) {
        printf("Robot x mm: %f\r\n", (double)position_get_x_mm());
      }
      else if (!strncmp(arg, "y", ARG_LENGTH)) {
        printf("Robot y mm: %f\r\n", (double)position_get_y_mm());
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
      printf("  p <arg>:   Print internal value.\r\n");
      printf("             <arg> can be one of:\r\n");
      printf("             x: print robot's x position.\r\n");
      printf("             y: print robot's y position.\r\n");
      printf("  h: Display this help.\r\n");
    }
    else {
      printf("Unknown command '%c'. Type 'h' for help.\r\n", command);
    }

    vTaskDelay(1 / portTICK_RATE_MS);
  }
}
