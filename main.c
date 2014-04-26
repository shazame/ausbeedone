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
#include "init.h"
#include "asserv_manager.h"

// Private function prototypes
void blink1();
void coucou();
// Global variables
//static xSemaphoreHandle xTestSemaphore = NULL;
struct ausbee_l298_chip mot_droit, mot_gauche;

xSemaphoreHandle CANReceiveSemaphore;
CanRxMsg CAN_RxStruct;
uint32_t count_sum = 0;

void TIM8_UP_TIM13_IRQHandler(void)
{
  uint32_t count;

  if (TIM_GetITStatus(TIM8, TIM_IT_Update) == SET) {
    count = TIM3->CNT;
    count_sum += count;
    TIM_SetCounter(TIM3, 0);
    //ausbee_l298_set_duty_cycle(l298_chip, ausbee_eval_pid(&pid, count));
    TIM_ClearFlag(TIM8, TIM_FLAG_Update);
  }
}

int main(void) {
  // Call the platform initialization function
  platform_init_HSE_PLL();
  platform_init_USART(USART_DEBUG, 115200);
  platform_init_LED();

  // Encoder setup
  platform_encoder2_init();
  ausbee_encoder_init_timer(TIM3);
  ausbee_init_sampling_timer(TIM8, 16800, 1000);

  // Init motors
  init_mot(&mot_droit, &mot_gauche);

  // Launching control system
  struct ausbee_cs cs;
  //start_control_system(&cs);
  // TODO: passer la main au traj manager

  xTaskCreate(blink1, (const signed char *)"LED1", 340, NULL, 1, NULL );

  // Go forward, with half power
  ausbee_l298_set_duty_cycle(mot_droit, 100);

  vTaskStartScheduler();

  for(;;) {

  }

  return 0;
}

void blink1() {
  /* Block for 500ms. */
  //const TickType_t xDelay = 500 / portTICK_PERIOD_MS;

  for(;;) {
    //platform_toggle_led(PLATFORM_LED0);
    printf("count: %d\r\n", TIM3->CNT);
    printf("count sum: %d\r\n", count_sum);
    vTaskDelay(1 * portTICK_RATE_MS);
  }
}

//#define init_servo(servo, NUM) \
//{
//  servo.TIMx = SERVO##NUM##_TIM;
//
