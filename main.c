#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "stm32f4xx_conf.h"
#include "FreeRTOS.h"
#include "list.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"
#include "platform.h"
#include "misc.h"

#include <AUSBEE/l298_driver.h>

#include "utils/define.h"
#include "utils/init.h"
#include "utils/actions.h"
#include "utils/motors_wrapper.h"
#include "utils/position_manager.h"
#include "utils/control_system_debug.h"
#include "utils/gp2_detect.h"

#include "demo/demo_square.h"
#include "demo/demo_square_reverse.h"
#include "demo/demo_circle.h"
#include "demo/demo_strat.h"
#include "demo/demo_fresque.h"
#include "demo/demo_yellow_side_strategy.h"
#include "demo/demo_red_side_strategy.h"

#include "control_system.h"
#include "trajectory_manager.h"

#include "cli.h"

// Private function prototypes
void blink1();
void strategy_start();
// Global variables
struct control_system am;
struct trajectory_manager t;
struct ausbee_l298_chip right_mot, left_mot;

int32_t right_motor_ref = 0;
int32_t left_motor_ref = 0;

volatile unsigned char buffer[AUSBEE_LIDAR_PICCOLO_FRAME_LENGTH];
volatile uint8_t elapsed_time = 0;
xSemaphoreHandle USART1ReceiveHandle;
xSemaphoreHandle CANReceiveSemaphore;
CanRxMsg CAN_RxStruct;

int main(void)
{
  // Call the platform initialization function
  platform_hse_pll_init();
  platform_usart_init(USART_DEBUG, 115200);
  platform_led_init();

  init_can();
  init_servo_position_depart();
  init_gpio_robot();
  init_turbine();
  init_timer_relais();

  // Encoders setup
  init_encoders();

  // Init motors
  init_mot(&left_mot, &right_mot);

  // Init motors_wrapper
  motors_wrapper_init(&left_mot, &right_mot);

  // Setting up position manager
  position_init();
  position_set_tick_per_meter(25665);
  position_set_axle_track_mm(357);

  // Launching control system
  control_system_start(&am);
  // With debug output
  control_system_debug_start(&am);

  // Launching trajectory manager
  trajectory_init(&t, &am);
  trajectory_start(&t);

  // Starting detection system
  gp2_detect_start(&t, &am);

  xTaskCreate(blink1, (const signed char *)"LED1", 100, NULL, 1, NULL );

  // Launching command line interface
  //cli_start(&t);

  // Launching a demonstration
  //demo_square_start(&t);
  //demo_square_reverse_start(&t);
  //demo_circle_start(&t);
  //demo_strat_start(&t);
  //demo_fresque_start(&t);

  strategy_start();

  vTaskStartScheduler();

  for (;;) {

  }

  return 0;
}

void blink1(void)
{
  for (;;) {
    platform_led_toggle(PLATFORM_LED0);

    vTaskDelay(1000 / portTICK_RATE_MS);
  }
}

void strategy_start(void)
{
  while(!presence_tirette())
    ;
  while(presence_tirette())
    ;

  elapsed_time = 0;

  if(couleur_depart()==COULEUR_JAUNE)
  {
    demo_yellow_side_strategy_start(&t);
  }
  else if(couleur_depart()==COULEUR_ROUGE)
  {
    demo_red_side_strategy_start(&t);
  }
}
