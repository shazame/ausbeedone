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
#include <AUSBEE/encoder.h>

#include "utils/init.h"
#include "utils/motors_wrapper.h"
#include "utils/position_manager.h"

#include "demo/demo_square.h"

#include "control_system.h"

#include "cli.h"

// Private function prototypes
void blink1();
// Global variables
struct control_system am;
struct ausbee_l298_chip right_mot, left_mot;

int32_t right_motor_ref = 0;
int32_t left_motor_ref = 0;

volatile unsigned char buffer[AUSBEE_LIDAR_PICCOLO_FRAME_LENGTH];
xSemaphoreHandle USART1ReceiveHandle;
xSemaphoreHandle CANReceiveSemaphore;
CanRxMsg CAN_RxStruct;

int main(void)
{
  // Call the platform initialization function
  platform_hse_pll_init();
  platform_usart_init(USART_DEBUG, 115200);
  platform_led_init();

  // Encoder setup
  ausbee_encoder_clock_enable(TIM8);
  ausbee_init_sampling_timer(TIM8, 16800, 1000);

  // Right encoder
  platform_encoder1_init();
  ausbee_encoder_clock_enable(TIM1);
  ausbee_encoder_init_timer(TIM1);

  // Left encoder
  platform_encoder2_init();
  ausbee_encoder_clock_enable(TIM3);
  ausbee_encoder_init_timer(TIM3);

  // Init motors
  init_mot(&left_mot, &right_mot);

  // Init motors_wrapper
  motors_wrapper_init(&left_mot, &right_mot);

  // Setting up position manager
  position_init();
  position_set_tick_per_meter(1170);
  position_set_axle_track_mm(235);

  // Launching control system
  control_system_start(&am);

  xTaskCreate(blink1, (const signed char *)"LED1", 140, NULL, 1, NULL );

  // Launching command line interface
  //cli_start(&am);

  // Launching a demonstration
  demo_square_start(&am);

  vTaskStartScheduler();

  for(;;) {

  }

  return 0;
}

void blink1(void)
{
  for(;;) {
    platform_led_toggle(PLATFORM_LED0);

    vTaskDelay(1000 / portTICK_RATE_MS);
  }
}
