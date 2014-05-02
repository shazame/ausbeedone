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
#include "utils/encoders.h"
#include "asserv_manager.h"
#include "trajectory_manager.h"

// Private function prototypes
void blink1();
// Global variables
struct trajectory_manager traj;
struct asserv_manager am;
struct ausbee_l298_chip mot_droit, mot_gauche;

volatile unsigned char buffer[AUSBEE_LIDAR_PICCOLO_FRAME_LENGTH];
xSemaphoreHandle USART1ReceiveHandle;
xSemaphoreHandle CANReceiveSemaphore;
CanRxMsg CAN_RxStruct;

void TIM8_UP_TIM13_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM8, TIM_IT_Update) == SET) {
    set_left_encoder_value(TIM1->CNT);
    set_right_encoder_value(TIM3->CNT);
    //TIM_SetCounter(TIM3, 0);
    TIM_ClearFlag(TIM8, TIM_FLAG_Update);
  }
}

int main(void) {
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
  init_mot(&mot_droit, &mot_gauche);

  // Init trajectory manager
  set_motors(&traj, &mot_droit, &mot_gauche);

  // Launching control system
  start_control_system(&am, &traj);
  // TODO: passer la main au traj manager

  xTaskCreate(blink1, (const signed char *)"LED1", 340, NULL, 1, NULL );

  vTaskStartScheduler();

  control_system_set_right_motor_ref(&am, 1000);

  for(;;) {

  }

  return 0;
}

void blink1(void) {
  /* Block for 500ms. */
  //const TickType_t xDelay = 500 / portTICK_PERIOD_MS;

  for(;;) {
    platform_led_toggle(PLATFORM_LED0);
    vTaskDelay(10 * portTICK_RATE_MS);
  }
}
