#include <stdio.h>
#include <stdlib.h>

#include "init.h"
#include "FreeRTOS.h"
#include "list.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"
#include "actions.h"

#include "define.h"

#include <AUSBEE/encoder.h>

extern xSemaphoreHandle USART1ReceiveHandle;

void init_usart_interrupt()
{
  NVIC_InitTypeDef NVIC_InitStructure;
  USART_ITConfig(USART1,USART_IT_RXNE, ENABLE);
  NVIC_InitStructure.NVIC_IRQChannel=USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
  NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void init_can_rx_interrupt()
{
  NVIC_InitTypeDef NVIC_InitStructure;
  CAN_ITConfig(CAN1,CAN_IT_FMP0, ENABLE);
  NVIC_InitStructure.NVIC_IRQChannel=CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
  NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}

void init_can()
{
  platform_can_init(CAN1);
  CAN_InitTypeDef CAN_InitTypeDef_1;
  CAN_FilterInitTypeDef CAN_FilterInitStructure;
  CAN_StructInit(&CAN_InitTypeDef_1);
  CAN_InitTypeDef_1.CAN_Prescaler = 336;
  CAN_InitTypeDef_1.CAN_Mode = CAN_Mode_Normal;
  CAN_InitTypeDef_1.CAN_AWUM = ENABLE;
  CAN_InitTypeDef_1.CAN_TXFP = ENABLE;
  if(CAN_Init(CAN1, &CAN_InitTypeDef_1) == CAN_InitStatus_Failed) {
    platform_led_set(PLATFORM_LED2);
    while(1);
  }

  // CAN filter init

  CAN_FilterInitStructure.CAN_FilterNumber=0;
  CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_16bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh=0x0001;
  CAN_FilterInitStructure.CAN_FilterIdLow=0x0001;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0001;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0001;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment=0;
  CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);
}

void init_mot(struct ausbee_l298_chip* mot2, struct ausbee_l298_chip* mot1)
{
  mot2->timer_channel=2;
  mot2->gpio_enable_pin=PLATFORM_ENABLE_MOTOR2_PIN;
  mot2->gpio_dir_pin=PLATFORM_DIR_MOTOR2_PIN;
  mot2->pwm_frequency=10000;
  mot2->gpio_dir_port=PLATFORM_DIR_MOTOR2_PORT;
  mot2->gpio_enable_port=PLATFORM_ENABLE_MOTOR2_PORT;
  mot2->TIMx=TIM9;
  enum AUSBEE_L298_DRIVER_ERROR error;
  platform_motor2_init_io();
  error=ausbee_l298_init_chip(*mot2);
  //if (error==ENO_ERROR)
  //  platform_led_toggle(PLATFORM_LED1);
  ausbee_l298_enable_chip(*mot2, 1);

  mot1->timer_channel=1;
  mot1->gpio_enable_pin=PLATFORM_ENABLE_MOTOR1_PIN;
  mot1->gpio_dir_pin=PLATFORM_DIR_MOTOR1_PIN;
  mot1->pwm_frequency=10000;
  mot1->gpio_dir_port=PLATFORM_DIR_MOTOR1_PORT;
  mot1->gpio_enable_port=PLATFORM_ENABLE_MOTOR1_PORT;
  mot1->TIMx=TIM9;
  platform_motor1_init_io();
  error=ausbee_l298_init_chip(*mot1);
  //if (error==ENO_ERROR)
  //  platform_led_toggle(PLATFORM_LED2);
  ausbee_l298_enable_chip(*mot1, 1);

}

#ifdef ENCODERS_HAVE_QUADRATURE
  #define USE_ENCODER_QUADRATURE 1
#else
  #define USE_ENCODER_QUADRATURE 0
#endif
void init_encoders(void)
{
  ausbee_encoder_clock_enable(TIM8);
  ausbee_init_sampling_timer(TIM8, 16800, 1000);

  platform_encoder_init();

  // Right encoder
  ausbee_encoder_clock_enable(TIM1);
  ausbee_encoder_init_timer(TIM1, USE_ENCODER_QUADRATURE);

  // Left encoder
  ausbee_encoder_clock_enable(TIM3);
  ausbee_encoder_init_timer(TIM3, USE_ENCODER_QUADRATURE);
}

/* initialise la pwm pour la turbine du canon
 * la turbine doit etre branché sur port servo1
 */
void init_turbine(void)
{
  // control structures
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);  // give clock to the GPIO

  platform_gpio_init(GPIO1, GPIO_OType_PP, GPIO_Mode_OUT, GPIO_Speed_50MHz, GPIO_PuPd_NOPULL);                                 // init gpio for turbine enable

  platform_gpio_set(GPIO1);                              // desable turbine

  TIM_TimeBaseInitTypeDef TimeBaseInit_PWM;
  GPIO_InitTypeDef InitTypeDef_PWM;

  TIM_TimeBaseStructInit(&TimeBaseInit_PWM);             // initialize the struct

  TimeBaseInit_PWM.TIM_Prescaler=1000;                   // 168MHz/168Khz
  TimeBaseInit_PWM.TIM_CounterMode = TIM_CounterMode_Up; // counter mode up
  TimeBaseInit_PWM.TIM_Period = 560;                     // 50Khz/300Hz  //560 pour 300
  TimeBaseInit_PWM.TIM_ClockDivision = 0x0000;           // is not used
  TimeBaseInit_PWM.TIM_RepetitionCounter = 0x0000;       // is not used
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10,ENABLE);   // Enable APB2 clock on TIM10
  TIM_TimeBaseInit(TIM10, &TimeBaseInit_PWM);            // Initialize TIM10
  TIM_Cmd(TIM10, ENABLE);

  // IO
  GPIO_StructInit(&InitTypeDef_PWM);
  InitTypeDef_PWM.GPIO_Pin = Pin_PWM_GPIOF;
  InitTypeDef_PWM.GPIO_Speed = GPIO_Speed_50MHz;         // Output maximum frequency at 100MHz
  InitTypeDef_PWM.GPIO_Mode = GPIO_Mode_AF;              // Mode alternate function
  //InitTypeDef_PWM.GPIO_OType = GPIO_OType_PP;          // Mode Push-Pull
  InitTypeDef_PWM.GPIO_PuPd = GPIO_PuPd_NOPULL;          // No pull-up/ pull down
  GPIO_Init(GPIOF, &InitTypeDef_PWM);                    // Initialize PWM Pin on GPIOF
  GPIO_PinAFConfig(GPIOF,GPIO_PinSource6,GPIO_AF_TIM10);

  TIM_OCInitTypeDef OCInit_PWM;                          // Create a OCInitTypeDef for PWM
  TIM_OCStructInit(&OCInit_PWM);
  OCInit_PWM.TIM_OCMode = TIM_OCMode_PWM1;               // Set PWM Mode
  OCInit_PWM.TIM_Pulse = 280;                            // 280=Duty cycle to 50%
  OCInit_PWM.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OC1Init(TIM10, &OCInit_PWM);                       // Initialize OSC1
}

void init_lidar()
{
  platform_usart_init(USART1,115200);
  init_usart_interrupt();

  //init timer used to know if the obstacle is still in front of the robot
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);  // give clock to the TIM7

  TIM_TimeBaseInitTypeDef TimeBaseInit_lidar;
  TIM_TimeBaseStructInit(&TimeBaseInit_lidar);  // initialize the struct

  //timer is initialize to count 1s
  TimeBaseInit_lidar.TIM_Prescaler=10000;                    // 168MHz/168Khz
  TimeBaseInit_lidar.TIM_CounterMode = TIM_CounterMode_Up;  // counter mode up
  TimeBaseInit_lidar.TIM_Period = 16800;                    // To count 1 second 
  TimeBaseInit_lidar.TIM_ClockDivision = 0x0000;            // is not used
  TimeBaseInit_lidar.TIM_RepetitionCounter = 0x0000;        // is not used

  TIM_TimeBaseInit(TIM7, &TimeBaseInit_lidar);            // Initialize TIM2
  TIM_Cmd(TIM7, ENABLE);



  USART1ReceiveHandle=xSemaphoreCreateMutex();
  if(USART1ReceiveHandle== NULL)
  {
    //platform_led_toggle(PLATFORM_LED6);
  }
}

//Fonction pour initialiser les gpio en fonction du robot
void init_gpio_robot()
{
  //tirette
  platform_gpio_init(GPIO_TIRETTE, GPIO_OType_PP, GPIO_Mode_IN, GPIO_Speed_50MHz, GPIO_PuPd_DOWN);
  //couleur départ 
  platform_gpio_init(GPIO_SELECTION_COULEUR, GPIO_OType_PP, GPIO_Mode_IN, GPIO_Speed_50MHz, GPIO_PuPd_NOPULL);
  //Relais
  platform_gpio_init(GPIO_RELAIS, GPIO_OType_PP, GPIO_Mode_OUT, GPIO_Speed_50MHz, GPIO_PuPd_NOPULL);
  platform_gpio_init(GPIO1, GPIO_OType_PP, GPIO_Mode_OUT, GPIO_Speed_50MHz, GPIO_PuPd_NOPULL);
  platform_gpio_init(GPIO3, GPIO_OType_PP, GPIO_Mode_OUT, GPIO_Speed_50MHz, GPIO_PuPd_NOPULL);
  platform_gpio_init(GPIO4, GPIO_OType_PP, GPIO_Mode_OUT, GPIO_Speed_50MHz, GPIO_PuPd_NOPULL);
  platform_gpio_init(GPIO5, GPIO_OType_PP, GPIO_Mode_OUT, GPIO_Speed_50MHz, GPIO_PuPd_NOPULL);
  platform_gpio_init(GPIO7, GPIO_OType_PP, GPIO_Mode_OUT, GPIO_Speed_50MHz, GPIO_PuPd_NOPULL);
  platform_gpio_init(GPIO8, GPIO_OType_PP, GPIO_Mode_OUT, GPIO_Speed_50MHz, GPIO_PuPd_NOPULL);
}

//Fonction utilisée pour initialiser le timer chargé de compter les secondes
//Compteur initialisé pour compter une seconde
void init_timer_relais()
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);   // Enable APB1 clock on TIM2
  TIM_TimeBaseInitTypeDef TimeBaseInit_relais;
  TIM_TimeBaseStructInit(&TimeBaseInit_relais);  // initialize the struct

  TimeBaseInit_relais.TIM_Prescaler=10000;                    // 168MHz/168Khz
  TimeBaseInit_relais.TIM_CounterMode = TIM_CounterMode_Up;  // counter mode up
  TimeBaseInit_relais.TIM_Period = 16800;                    // To count 1 second 
  TimeBaseInit_relais.TIM_ClockDivision = 0x0000;            // is not used
  TimeBaseInit_relais.TIM_RepetitionCounter = 0x0000;        // is not used

  TIM_TimeBaseInit(TIM2, &TimeBaseInit_relais);            // Initialize TIM2
  NVIC_EnableIRQ(TIM2_IRQn);
  TIM_Cmd(TIM2, ENABLE);
}
