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

#include <AUSBEE/lidar.h>

#include "define.h"
#include "position_manager.h"
#include "motors_wrapper.h"
#include "actions.h"

// extern global variables
//usart
extern volatile unsigned char buffer[AUSBEE_LIDAR_PICCOLO_FRAME_LENGTH];
extern xSemaphoreHandle USART1ReceiveHandle;
//CAN
extern xSemaphoreHandle CANReceiveSemaphore;
extern CanRxMsg CAN_RxStruct;
extern volatile uint8_t elapsed_time;

void USART1_IRQHandler(void)
{
	// variable to know if the frame is at the beginning 
	static uint8_t not_started=1;

	//Index of the buffer 
	static uint8_t index_buffer=0;

	//receive variable
	static uint16_t rec;
	
	//verifie le flag d'interruption
	if(USART_GetITStatus(USART1,USART_IT_RXNE) == SET)
	{
		//compulse the value received
		rec=USART_ReceiveData(USART1);
		//if the beginning of the frame has not been reached
		if(not_started==1)
		{
			//check if the frame is at the beginning
			if(rec==0xFA)
			{
				not_started=0;
				//add the element to buffer and increment the buffer
				buffer[0]=(unsigned char)rec;
				index_buffer++;
			}
		}
		else
		{
			//add the element
			buffer[index_buffer]=(unsigned char)rec;
			index_buffer++;
		}
		//reset variables
		if(index_buffer==AUSBEE_LIDAR_PICCOLO_FRAME_LENGTH)
		{
			xSemaphoreGive(USART1ReceiveHandle);
			index_buffer=0;
			not_started=1;
		}

	}
}

void CAN1_RX0_IRQHandler(void) {
	// Check the cause of the interrupt
	if(CAN_GetITStatus(CAN1, CAN_IT_FMP0) == SET) { // Pending message in Fifo 0
		CAN_Receive(CAN1, CAN_FIFO0, &CAN_RxStruct);
		xSemaphoreGive(CANReceiveSemaphore);
	}
	else if(CAN_GetITStatus(CAN1, CAN_IT_FF0) == SET) { // Fifo 0 is full
	}
	else if(CAN_GetITStatus(CAN1, CAN_IT_FOV0) == SET) { // Fifo 0 overrun
	}
}

// Updating encoder value
void TIM8_UP_TIM13_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM8, TIM_IT_Update) == SET) {
#ifdef ENCODERS_HAVE_QUADRATURE
    // Specific operations have to be done to read encoder's value has negative
    // when moving backward
    uint16_t left_counter = TIM1->CNT;
    int16_t left_encoder_diff = *(int16_t *)(&left_counter);
    uint16_t right_counter = TIM3->CNT;
    int16_t right_encoder_diff = -*(int16_t *)(&right_counter);
#else
    // Reading encoder value
    int32_t left_encoder_diff = TIM3->CNT;
    int32_t right_encoder_diff = TIM1->CNT;

    // If a motor is moving backward, its encoder value
    // is read as a negative number
    if (!motors_wrapper_left_motor_is_moving_forward()) {
      left_encoder_diff  = -left_encoder_diff;
    }
    if (!motors_wrapper_right_motor_is_moving_forward()) {
      right_encoder_diff = -right_encoder_diff;
    }
#endif

    // Updating position
    position_update(left_encoder_diff, right_encoder_diff);

    // Resetting encoders value
    TIM_SetCounter(TIM1, 0);
    TIM_SetCounter(TIM3, 0);
    TIM_ClearFlag(TIM8, TIM_FLAG_Update);
  }
}

// Count one second
void TIM2_IRQHandler(void)
{
  if(TIM_GetITStatus(TIM2,TIM_IT_Update) == SET)
  {
    elapsed_time++;
    if (elapsed_time>=88)
    {
      //disable_power_relay();
      platform_gpio_set(GPIO_RELAIS);
      platform_led_toggle(PLATFORM_LED5);
    }
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
  }
}
