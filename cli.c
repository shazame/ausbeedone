#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "utils/actions.h"
#include "utils/position_manager.h"
#include "cli.h"

#include "demo/demo_yellow_side_strategy.h"
#include "demo/demo_red_side_strategy.h"
#include "demo/demo_pid.h"

char cli_last_buffer[CLI_BUFFER_SIZE] = {0};

void cli_task(void *);
void cli_execute(char *cmd, struct trajectory_manager *t, uint8_t argc, char **argv);

void cli_start(struct trajectory_manager *t)
{
  xTaskCreate(cli_task, (const signed char *)"CLI", 200, (void *)t, 1, NULL );
}

static void cli_getline(char *buff)
{
  int c = 0;
  int i = 0;

  memset(buff, 0, CLI_BUFFER_SIZE * sizeof(char));

  while (((c = getchar()) != EOF) && ((char)c != CLI_END_CHAR)) {
    if ((char)c == CLI_DEL_CHAR) {
      if (i > 0) {
        buff[--i] = 0;
        printf("\b \b");
      }
    }
    else if ((char)c == CLI_ESC_CHAR) {
      // ANSI Escape sequence detected
      if ((char)getchar() != 91) break;
      // Get arrow key id
      c = getchar();
      switch (c) {
        case CLI_UP_KEY:
          if (cli_last_buffer != NULL) {
            memset(buff, 0, CLI_BUFFER_SIZE * sizeof(char));
            strncpy(buff, cli_last_buffer, CLI_BUFFER_SIZE);
            for (; i > 0; i--) {
              printf("\b \b");
            }
            printf("%s", buff);
            i = strlen(buff);
          }
          break;
        case CLI_DOWN_KEY:
          break;
        case CLI_RIGHT_KEY:
          if (i < CLI_BUFFER_SIZE-1 && buff[i] != 0) {
            printf("%c", buff[i]);
            i++;
          }
          break;
        case CLI_LEFT_KEY:
          if (i > 0) {
            i--;
            printf("\b");
          }
          break;
        default:
          printf("Unknown escape sequence.\r\n");
      }
    }
    else if (i < CLI_BUFFER_SIZE-1) {
      buff[i++] = (char)c;
      printf("%c", (char)c);
    }
  }
  // If enter key is pressed before any char is typed, last command is sent
  if ((buff[0] == 0) && (cli_last_buffer != NULL)) {
    strncpy(buff, cli_last_buffer, CLI_BUFFER_SIZE);
    //printf("%s", buff);
  }
  printf("\r\n");
}

void cli_task(void *data)
{
  struct trajectory_manager *t = (struct trajectory_manager *)data;

  char cli_buffer[CLI_BUFFER_SIZE];
  uint8_t argc = 0;
  char *cmd;
  char *argv[CLI_MAX_ARGS+1];

  for (;;) {
    printf("$ ");
    cli_getline(cli_buffer);
    strncpy(cli_last_buffer, cli_buffer, CLI_BUFFER_SIZE);

    // CLI buffer tokenization
    cmd = strtok(cli_buffer, CLI_DELIMITER);
    for (argc = 0; cmd != NULL; argc++) {
      argv[argc] = strtok(NULL, CLI_DELIMITER);
      if (argv[argc] == NULL || argc >= CLI_MAX_ARGS)
        break;
    }

    if (cmd == NULL) {
      printf("Read line is empty. Type 'help' for help.\r\n");
    }
    else {
      cli_execute(cmd, t, argc, argv);
    }

    vTaskDelay(1 / portTICK_RATE_MS);
  }
}

static int8_t cli_streq(char *str1, char*str2)
{
  return !strncmp(str1, str2, CLI_BUFFER_SIZE);
}

void cli_execute_goto_d(struct trajectory_manager *t, uint8_t argc, char **argv)
{
  if (argc < 1) {
    printf("Error: the command 'd' needs 1 argument.\r\n");
  }

  float distance = strtof(argv[0], NULL);
  trajectory_goto_d_mm(t, distance);
  printf("Distance: %f\r\n", (double)distance);
}

void cli_execute_goto_a(struct trajectory_manager *t, uint8_t argc, char **argv)
{
  if (argc < 1) {
    printf("Error: the command 'a' needs 1 argument.\r\n");
  }

  float angle = strtof(argv[0], NULL);
  trajectory_goto_a_rel_deg(t, angle);
  printf("Angle: %f\r\n", (double)angle);
}

void cli_execute_get(struct trajectory_manager *t, uint8_t argc, char **argv)
{
  if (argc < 1) {
    printf("Error: the command 'get' needs at least 1 argument.\r\n");
  }

  uint8_t i = 0;
  for (; i < argc; i++) {
    if (cli_streq(argv[i], "x")) {
      printf("Robot x mm: %f\r\n", (double)position_get_x_mm());
    }
    else if (cli_streq(argv[i], "y")) {
      printf("Robot y mm: %f\r\n", (double)position_get_y_mm());
    }
    else if (cli_streq(argv[i], "a")) {
      printf("Robot angle deg: %f\r\n", (double)position_get_angle_deg(NULL));
    }
    else if (cli_streq(argv[i], "d")) {
      printf("Robot distance mm: %f\r\n", (double)position_get_distance_mm(NULL));
    }
    else if (cli_streq(argv[i], "encr")) {
      printf("Right encoder value: %f\r\n", (double)position_get_right_encoder(NULL));
    }
    else if (cli_streq(argv[i], "encl")) {
      printf("Left encoder value: %f\r\n", (double)position_get_left_encoder(NULL));
    }
    else if (cli_streq(argv[i], "cur_id")) {
      printf("Position manager cur_id: %"PRId32"\r\n", trajectory_get_cur_id(t));
    }
    else if (cli_streq(argv[i], "last_id")) {
      printf("Position manager last_id: %"PRId32"\r\n", trajectory_get_last_id(t));
    }
    else if (cli_streq(argv[i], "pid_dump")) {
      printf("Distance PID: %f, %f, %f\r\n", (double)ausbee_pid_get_kp(&(t->cs->pid_distance)),
          (double)ausbee_pid_get_ki(&(t->cs->pid_distance)),
          (double)ausbee_pid_get_kd(&(t->cs->pid_distance)));
      printf("Angle PID:    %f, %f, %f\r\n", (double)ausbee_pid_get_kp(&(t->cs->pid_angle)),
          (double)ausbee_pid_get_ki(&(t->cs->pid_angle)),
          (double)ausbee_pid_get_kd(&(t->cs->pid_angle)));
    }
    else {
      printf("Invalid argument '%s'.\r\n", argv[i]);
    }
  }
}

void cli_execute_set(struct trajectory_manager *t, uint8_t argc, char **argv)
{
  if (argc < 2) {
    printf("Error: the command 'set' needs 2 argument.\r\n");
  }

  float value = strtof(argv[1], NULL);

  if (cli_streq(argv[0], "speed_high")) {
    control_system_set_speed_high(t->cs);
    printf("Max speed.\r\n");
  }
  else if (cli_streq(argv[0], "speed_medium")) {
    control_system_set_speed_medium(t->cs);
    printf("Medium speed.\r\n");
  }
  else if (cli_streq(argv[0], "speed_low")) {
    control_system_set_speed_low(t->cs);
    printf("Low speed.\r\n");
  }
  else if (cli_streq(argv[0], "speed")) {
    control_system_set_speed_ratio(t->cs, value);
    printf("Speed: %f\r\n", (double)value);
  }
  else if (cli_streq(argv[0], "pid_d_P")) {
    ausbee_pid_set_kp(&(t->cs->pid_distance), value);
    printf("Distance P: %f\r\n", (double)value);
  }
  else if (cli_streq(argv[0], "pid_d_I")) {
    ausbee_pid_set_ki(&(t->cs->pid_distance), value);
    printf("Distance I: %f\r\n", (double)value);
  }
  else if (cli_streq(argv[0], "pid_d_D")) {
    ausbee_pid_set_kd(&(t->cs->pid_distance), value);
    printf("Distance D: %f\r\n", (double)value);
  }
  else if (cli_streq(argv[0], "pid_a_P")) {
    ausbee_pid_set_kp(&(t->cs->pid_angle), value);
    printf("Angle P: %f\r\n", (double)value);
  }
  else if (cli_streq(argv[0], "pid_a_I")) {
    ausbee_pid_set_ki(&(t->cs->pid_angle), value);
    printf("Angle I: %f\r\n", (double)value);
  }
  else if (cli_streq(argv[0], "pid_a_D")) {
    ausbee_pid_set_kd(&(t->cs->pid_angle), value);
    printf("Angle D: %f\r\n", (double)value);
  }
  else if (cli_streq(argv[0], "axle_track")) {
    position_set_axle_track_mm(value);
    printf("Axle track: %f\r\n", (double)value);
  }
  else {
    printf("Invalid argument '%s'.\r\n", argv[0]);
  }
}

void cli_execute_start(struct trajectory_manager *t, uint8_t argc, char **argv)
{
  if (argc < 1) {
    printf("Error: the command 'start' needs 1 argument.\r\n");
  }

  if (cli_streq(argv[0], "pid")) {
    demo_pid_start(t, 1);
    printf("Starting PID test.\r\n");
  }
  else if (cli_streq(argv[0], "pidi")) {
    demo_pid_start(t, -1);
    printf("Starting PID test.\r\n");
  }
  else if (cli_streq(argv[0], "ys")) {
    demo_yellow_side_strategy_start(t);
    printf("Starting strategy for yellow side.\r\n");
  }
  else if (cli_streq(argv[0], "rs")) {
    demo_red_side_strategy_start(t);
    printf("Starting strategy for red side.\r\n");
  }
  else {
    printf("Invalid argument '%s'.\r\n", argv[0]);
  }
}

void cli_execute_stop(uint8_t argc, char **argv)
{
  if (argc < 1) {
    printf("Error: the command 'stop' needs 1 argument.\r\n");
  }

  if (cli_streq(argv[0], "pid") || cli_streq(argv[0], "pidi")) {
    demo_pid_stop();
    printf("Ending PID test.\r\n");
  }
  else if (cli_streq(argv[0], "ys")) {
    demo_yellow_side_strategy_stop();
    printf("Ending strategy for yellow side.\r\n");
  }
  else if (cli_streq(argv[0], "rs")) {
    demo_red_side_strategy_stop();
    printf("Ending strategy for red side.\r\n");
  }
  else {
    printf("Invalid argument '%s'.\r\n", argv[0]);
  }
}

void cli_execute_call(uint8_t argc, char **argv)
{
}

void cli_execute_help()
{
  printf("Help:\r\n");
  printf("  Available commands are:\r\n");
  printf("  d <value>: Go forward/backward with the specified distance in mm.\r\n");
  printf("  a <value>: Rotate with the specified angle in degrees.\r\n");
  printf("  start/stop <arg>: Start/stop a task.\r\n");
  printf("             <arg> can be one of:\r\n");
  printf("             ys: Yellow strategy task.\r\n");
  printf("             rs: Red strategy task.\r\n");
  printf("             pid: PID test task.\r\n");
  printf("  get <arg1> <arg2> ...: Print internal values.\r\n");
  printf("             <argx> can be one of:\r\n");
  printf("             x:        print robot's x position.\r\n");
  printf("             y:        print robot's y position.\r\n");
  printf("             a:        print robot's angle.\r\n");
  printf("             d:        print robot's distance.\r\n");
  printf("             encl:    print left encoder's value.\r\n");
  printf("             encr:    print right encoder's value.\r\n");
  printf("             cur_id:   print position manager's current point id.\r\n");
  printf("             last_id:  print position manager's last point id.\r\n");
  printf("             pid_dump: print PID.\r\n");
  printf("  set <arg> <value>: Set internal value.\r\n");
  printf("             <value> should be a float.\r\n");
  printf("             <arg> can be one of:\r\n");
  printf("             speed_high:   set highest translation and rotation speed.\r\n");
  printf("             speed_medium: set medium translation and rotation speed.\r\n");
  printf("             speed_low:    set low translation and rotation speed.\r\n");
  printf("             speed :       set translation and rotation speed ratio to value (0 <= value <= 1).\r\n");
  printf("             pid_d_P :     set distance PID proportional value.\r\n");
  printf("             pid_d_I :     set distance PID integral value.\r\n");
  printf("             pid_d_D :     set distance PID derivative value.\r\n");
  printf("             pid_a_P :     set angle PID proportional value.\r\n");
  printf("             pid_a_I :     set angle PID integral value.\r\n");
  printf("             pid_a_D :     set angle PID derivative value.\r\n");
  printf("             axle_track :  set axle track in mm.\r\n");
  //printf("  m <arg> <arg2> : move an actuator\r\n");
  //printf("             <arg> can be one of: \r\n");
  //printf("             arm_l: left_arm \r\n");
  //printf("             arm_r: right_arm \r\n");
  //printf("                <arg2> can be one of: \r\n");
  //printf("                close: close the arm \r\n");
  //printf("                open: open the arm \r\n");
  //printf("             paint_r: right_paint \r\n");
  //printf("             paint_l: left_paint \r\n");
  //printf("                <arg2> can be one of: \r\n");
  //printf("                put: put the paint on the \"fresque\" \r\n");
  //printf("                release: release the paint on the \"fresque\" \r\n");
  printf("  help: Display this help.\r\n");
}

void cli_execute(char *cmd, struct trajectory_manager *t, uint8_t argc, char **argv)
{
  if (cli_streq(cmd, "d")) {          // Translate by given distance
    cli_execute_goto_d(t, argc, argv);
  }
  else if (cli_streq(cmd, "a")) {     // Rotate by given angle
    cli_execute_goto_a(t, argc, argv);
  }
  else if (cli_streq(cmd, "get")) {   // get a variable
    cli_execute_get(t, argc, argv);
  }
  else if (cli_streq(cmd, "set")) {   // set a variable
    cli_execute_set(t, argc, argv);
  }
  else if (cli_streq(cmd, "start")) { // start a task
    cli_execute_start(t, argc, argv);
  }
  else if (cli_streq(cmd, "stop")) {  // stop a task
    cli_execute_stop(argc, argv);
  }
  else if (cli_streq(cmd, "call")) {  // call a function
    cli_execute_call(argc, argv);
  }
  else if (cli_streq(cmd, "help")) {  // call a function
    cli_execute_help();
  }
  else {
      printf("Unknown command '%s'. Type 'help' for help.\r\n", cmd);
  }
}

/*
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
  char arg2[ARG_LENGTH] = {0};
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
    else if (command == 'm' || command == 'r') {
      cli_getstr(arg, ARG_LENGTH);
      cli_getstr(arg2, ARG_LENGTH);
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
    else if (command == 'r'){
      if (!strncmp(arg, "pid", ARG_LENGTH)) {
        if (!strncmp(arg2, "start", ARG_LENGTH)) {
          demo_pid_start(t, 1);
          printf("Starting PID test.\r\n");
        }
        else if (!strncmp(arg2, "stop", ARG_LENGTH)) {
          demo_pid_stop();
          printf("Ending PID test.\r\n");
        }
        else {
          printf("Invalid argument '%s'.\r\n", arg2);
        }
      }
      else if (!strncmp(arg, "pidi", ARG_LENGTH)) {
        if (!strncmp(arg2, "start", ARG_LENGTH)) {
          demo_pid_start(t, -1);
          printf("Starting PID test.\r\n");
        }
        else if (!strncmp(arg2, "stop", ARG_LENGTH)) {
          demo_pid_stop();
          printf("Ending PID test.\r\n");
        }
        else {
          printf("Invalid argument '%s'.\r\n", arg2);
        }
      }
      else if (!strncmp(arg, "ys", ARG_LENGTH)) {
        if (!strncmp(arg2, "start", ARG_LENGTH)) {
          demo_yellow_side_strategy_start(t);
          printf("Starting strategy for yellow side.\r\n");
        }
        else if (!strncmp(arg2, "stop", ARG_LENGTH)) {
          demo_yellow_side_strategy_stop();
          printf("Ending strategy for yellow side.\r\n");
        }
        else {
          printf("Invalid argument '%s'.\r\n", arg2);
        }
      }
      else if (!strncmp(arg, "rs", ARG_LENGTH)) {
        if (!strncmp(arg2, "start", ARG_LENGTH)) {
          demo_red_side_strategy_start(t);
          printf("Starting strategy for red side.\r\n");
        }
        else if (!strncmp(arg2, "stop", ARG_LENGTH)) {
          demo_red_side_strategy_stop();
          printf("Ending strategy for red side.\r\n");
        }
        else {
          printf("Invalid argument '%s'.\r\n", arg2);
        }
      }
      else {
        printf("Invalid argument '%s'.\r\n", arg);
      }
    }
    else if (command == 's') {
      if (!strncmp(arg, "speed_high", ARG_LENGTH)) {
        control_system_set_speed_high(t->cs);
        printf("Max speed.\r\n");
      }
      else if (!strncmp(arg, "speed_medium", ARG_LENGTH)) {
        control_system_set_speed_medium(t->cs);
        printf("Medium speed.\r\n");
      }
      else if (!strncmp(arg, "speed_low", ARG_LENGTH)) {
        control_system_set_speed_low(t->cs);
        printf("Low speed.\r\n");
      }
      else if (!strncmp(arg, "speed", ARG_LENGTH)) {
        control_system_set_speed_ratio(t->cs, value);
        printf("Speed: %f\r\n", (double)value);
      }
      else if (!strncmp(arg, "pid_d_P", ARG_LENGTH)) {
        ausbee_pid_set_kp(&(t->cs->pid_distance), value);
        printf("Distance P: %f\r\n", (double)value);
      }
      else if (!strncmp(arg, "pid_d_I", ARG_LENGTH)) {
        ausbee_pid_set_ki(&(t->cs->pid_distance), value);
        printf("Distance I: %f\r\n", (double)value);
      }
      else if (!strncmp(arg, "pid_d_D", ARG_LENGTH)) {
        ausbee_pid_set_kd(&(t->cs->pid_distance), value);
        printf("Distance D: %f\r\n", (double)value);
      }
      else if (!strncmp(arg, "pid_a_P", ARG_LENGTH)) {
        ausbee_pid_set_kp(&(t->cs->pid_angle), value);
        printf("Angle P: %f\r\n", (double)value);
      }
      else if (!strncmp(arg, "pid_a_I", ARG_LENGTH)) {
        ausbee_pid_set_ki(&(t->cs->pid_angle), value);
        printf("Angle I: %f\r\n", (double)value);
      }
      else if (!strncmp(arg, "pid_a_D", ARG_LENGTH)) {
        ausbee_pid_set_kd(&(t->cs->pid_angle), value);
        printf("Angle D: %f\r\n", (double)value);
      }
      else if (!strncmp(arg, "axle_track", ARG_LENGTH)) {
        position_set_axle_track_mm(value);
        printf("Axle track: %f\r\n", (double)value);
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
      else if (!strncmp(arg, "pid_dump", ARG_LENGTH)) {
        printf("Distance PID: %f, %f, %f\r\n", (double)ausbee_pid_get_kp(&(t->cs->pid_distance)),
                                               (double)ausbee_pid_get_ki(&(t->cs->pid_distance)),
                                               (double)ausbee_pid_get_kd(&(t->cs->pid_distance)));
        printf("Angle PID:    %f, %f, %f\r\n", (double)ausbee_pid_get_kp(&(t->cs->pid_angle)),
                                               (double)ausbee_pid_get_ki(&(t->cs->pid_angle)),
                                               (double)ausbee_pid_get_kd(&(t->cs->pid_angle)));
      }
      else {
        printf("Invalid argument '%s'.\r\n", arg);
      }
    }
    else if (command == 'm'){
      if (!strncmp(arg, "arm_l", ARG_LENGTH)) {
        if (!strncmp(arg2, "open", ARG_LENGTH)) {
          ouvrir_bras_gauche();
          printf("Left arm open\r\n");
        }
        else if (!strncmp(arg2, "close", ARG_LENGTH)) {
          fermer_bras_gauche();
          printf("Left arm close\r\n");
        }
        else {
          printf("Invalid argument '%s'.\r\n", arg2);
        }
      }
      else if (!strncmp(arg, "arm_r", ARG_LENGTH)) {
        if (!strncmp(arg2, "open", ARG_LENGTH)) {
          ouvrir_bras_droit();
          printf("Left arm open\r\n");
        }
        else if (!strncmp(arg2, "close", ARG_LENGTH)) {
          fermer_bras_droit();
          printf("Left arm close\r\n");
        }
        else {
          printf("Invalid argument '%s'.\r\n", arg2);
        }
      }
      else if (!strncmp(arg, "paint_r", ARG_LENGTH)) {
        if (!strncmp(arg2, "put", ARG_LENGTH)) {
          placer_peinture_canon();
          printf("Left arm open\r\n");
        }
        else if (!strncmp(arg2, "release", ARG_LENGTH)) {
          init_servo_peinture_canon();
          printf("Left arm close\r\n");
        }
        else {
          printf("Invalid argument '%s'.\r\n", arg2);
        }
      }
      else if (!strncmp(arg, "paint_l", ARG_LENGTH)) {
        if (!strncmp(arg2, "put", ARG_LENGTH)) {
          placer_peinture_ausbee();
          printf("Left arm open\r\n");
        }
        else if (!strncmp(arg2, "release", ARG_LENGTH)) {
          init_servo_peinture_ausbee();
          printf("Left arm close\r\n");
        }
        else {
          printf("Invalid argument '%s'.\r\n", arg2);
        }
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
      printf("  r <arg> <arg2>: Start/stop a task.\r\n");
      printf("             <arg> can be one of:\r\n");
      printf("             ys: Yellow strategy task.\r\n");
      printf("             rs: Red strategy task.\r\n");
      printf("             pid: PID test task.\r\n");
      printf("             <arg2> can be one of:\r\n");
      printf("             start: Start the task.\r\n");
      printf("             stop : Stop the task.\r\n");
      printf("  s <arg> <value>:   Set internal value.\r\n");
      printf("             <value> should be a float.\r\n");
      printf("             <arg> can be one of:\r\n");
      printf("             speed_high:   set highest translation and rotation speed.\r\n");
      printf("             speed_medium: set medium translation and rotation speed.\r\n");
      printf("             speed_low:    set low translation and rotation speed.\r\n");
      printf("             speed :       set translation and rotation speed ratio to value (0 <= value <= 1).\r\n");
      printf("             pid_d_P :     set distance PID proportional value.\r\n");
      printf("             pid_d_I :     set distance PID integral value.\r\n");
      printf("             pid_d_D :     set distance PID derivative value.\r\n");
      printf("             pid_a_P :     set angle PID proportional value.\r\n");
      printf("             pid_a_I :     set angle PID integral value.\r\n");
      printf("             pid_a_D :     set angle PID derivative value.\r\n");
      printf("             axle_track :  set axle track in mm.\r\n");
      printf("  m <arg> <arg2> : move an actuator\r\n");
      printf("             <arg> can be one of: \r\n");
      printf("             arm_l: left_arm \r\n");
      printf("             arm_r: right_arm \r\n");
      printf("                <arg2> can be one of: \r\n");
      printf("                close: close the arm \r\n");
      printf("                open: open the arm \r\n");
      printf("             paint_r: right_paint \r\n");
      printf("             paint_l: left_paint \r\n");
      printf("                <arg2> can be one of: \r\n");
      printf("                put: put the paint on the \"fresque\" \r\n");
      printf("                release: release the paint on the \"fresque\" \r\n");
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
      printf("             pid_dump: print PID.\r\n");
      printf("  h: Display this help.\r\n");
    }
    else {
      printf("Unknown command '%c'. Type 'h' for help.\r\n", command);
    }

    vTaskDelay(1 / portTICK_RATE_MS);
  }
}
*/
