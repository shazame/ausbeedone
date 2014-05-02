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
#include "asserv_manager.h"

// Private function prototypes
void blink1();
void run_motors();
// Global variables
struct asserv_manager am;
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
  ausbee_init_sampling_timer(TIM8, 16800, 1000);

  // Left encoder
  platform_encoder1_init();
  ausbee_encoder_init_timer(TIM1);

  // Right encoder
  platform_encoder2_init();
  ausbee_encoder_init_timer(TIM3);

  // Init motors
  init_mot(&right_mot, &left_mot);

  // Init trajectory manager
  motors_wrapper_init(&right_mot, &left_mot);

  // Launching control system
  start_control_system(&am);
  // TODO: passer la main au traj manager

  xTaskCreate(blink1, (const signed char *)"LED1", 140, NULL, 1, NULL );
  xTaskCreate(run_motors, (const signed char *)"RunMotors", 140, NULL, 1, NULL );

  vTaskStartScheduler();

  for(;;) {

  }

  return 0;
}

void blink1(void)
{
  for(;;) {
    platform_led_toggle(PLATFORM_LED0);

    vTaskDelay(10 * portTICK_RATE_MS);
  }
}

void run_motors(void)
{
  for(;;) {
    control_system_set_right_motor_ref(&am, right_motor_ref);
    right_motor_ref += 500;

    control_system_set_left_motor_ref(&am, left_motor_ref);
    left_motor_ref += 300;

    vTaskDelay(50 * portTICK_RATE_MS);
  }
}
