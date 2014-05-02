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

#include "encoders.h"

// extern global variables
//usart
extern volatile unsigned char buffer[AUSBEE_LIDAR_PICCOLO_FRAME_LENGTH];
extern xSemaphoreHandle USART1ReceiveHandle;
//CAN
extern xSemaphoreHandle CANReceiveSemaphore;
extern CanRxMsg CAN_RxStruct;

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
    encoders_set_left_value(TIM1->CNT);
    encoders_set_right_value(TIM3->CNT);
    //TIM_SetCounter(TIM3, 0);
    TIM_ClearFlag(TIM8, TIM_FLAG_Update);
  }
}
