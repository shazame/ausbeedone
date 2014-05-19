#include <stdio.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"

#include "cli.h"

void cli_task(void *);

void cli_start(struct control_system *am)
{
  xTaskCreate(cli_task, (const signed char *)"CLI", 240, (void *)am, 1, NULL );
}

#define END_CHAR '\r'
static int uart_getchar(void)
{
  int c = getchar();

  if (c == END_CHAR) {
    printf("\r\n");
  }
  else {
    printf("%c", (char)c);
  }

  return c;
}

#define FLOAT_LENGTH 20
static float get_float(void)
{
  int c = 0;
  char buff[FLOAT_LENGTH] = {0};
  int i = 0;

  while (((c = uart_getchar()) != EOF) && ((char)c == ' '));

  do {
    buff[i++] = (char)c;

    if (i > FLOAT_LENGTH)
      break;

  } while (((c = uart_getchar()) != EOF) && ((char)c != ' ') && ((char) c != END_CHAR));

  if ((char) c != END_CHAR) {
    while (((c = uart_getchar()) != EOF) && ((char)c != END_CHAR));
  }

  float value = strtof(buff, NULL);

  return value;
}

void cli_task(void *data)
{
  struct control_system *am = (struct control_system *)data;

  char command = 0;
  float value = 0;
  int c = 0;

  for(;;) {
    value = 0;

    printf("$ ");
    command = uart_getchar();
    if (command == 'd' || command == 'a') {
      value = get_float();
    }
    else {
      while (((c = uart_getchar()) != EOF) && ((char)c != END_CHAR));
    }

    if (command == 'd') {
      control_system_set_distance_mm_ref(am, value);
      printf("Distance: %f\r\n", (double)value);
    }
    else if (command == 'a') {
      control_system_set_angle_deg_ref(am, value);
      printf("Angle: %f\r\n", (double)value);
    }
    else if (command == 'h') {
      printf("Help:\r\n");
      printf("  Available commands are:\r\n");
      printf("  d <float>: Go forward/backward with the specified distance in mm.\r\n");
      printf("  a <float>: Rotate with the specified angle in degrees.\r\n");
      printf("  h: Display this help.\r\n");
    }
    else {
      printf("Unknown command '%c'. Type 'h' for help.\r\n", command);
    }

    vTaskDelay(1 / portTICK_RATE_MS);
  }
}
