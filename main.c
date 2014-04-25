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
#include "init.h"

#include "AUSBEE/servo.h"

#define SERVO_FROM_MODULE_0 0x01	//canon bas
#define SERVO_FROM_MODULE_1 0x02	//canon haut
#define SERVO_FROM_MODULE_2 0x04  	//Bras gauche (coté ausbee) rentré :12 ouvert: 70
#define SERVO_FROM_MODULE_3 0x08	//Bras droit (alim) rentré: 91 ouvert : 35
#define SERVO_FROM_MODULE_4 0x10	//peinture ausbee sortie: 4 rentré: 52  
#define SERVO_FROM_MODULE_5 0x20	//peinture canon sortie: 80 rentré : 40
#define SERVO_FROM_MODULE_6 0x40
#define SERVO_FROM_MODULE_7 0x80

#define SERVO_CANON_BAS			SERVO_FROM_MODULE_0 	//fermé :79  ouvert: 50 
#define SERVO_CANON_HAUT		SERVO_FROM_MODULE_1	//fermé :24  ouvert: 62
#define SERVO_BRAS_GAUCHE		SERVO_FROM_MODULE_2
#define SERVO_BRAS_DROITE		SERVO_FROM_MODULE_3
#define SERVO_PEINTURE_COTE_AUSBEE	SERVO_FROM_MODULE_4
#define SERVO_PEINTURE_CANON		SERVO_FROM_MODULE_5


// Private function prototypes
void test();
void blink_led();
void test_lidar();
void move_zqsd();
void test_canon();
void init_servo();
void asserv_tempo();
void foutre_les_fresque();
void move_servo_from_servo_module(uint8_t servo, uint8_t angle);

volatile ausbee_lidar_data data[AUSBEE_LIDAR_PICCOLO_DATA_LENGTH];
volatile unsigned char buffer[AUSBEE_LIDAR_PICCOLO_FRAME_LENGTH];
xSemaphoreHandle USART1ReceiveHandle;
xSemaphoreHandle CANReceiveSemaphore;
CanRxMsg CAN_RxStruct;

ausbee_l298_chip mot_gauche;	//branché sur mot1
ausbee_l298_chip mot_droit;		//branché sur mot2
uint8_t enable_turbine=0;
ausbeeServo servo1, servo2, servo3, servo4;

int main(void) {
	// Call the platform initialization function
	platform_init_HSE_PLL();
	platform_init_USART(USART_DEBUG, 115200);
	platform_init_USART(USART1,115200);
	platform_init_LED();
	//platform_CAN_init(CAN1);
	//init_can();
	//init_turbine();
	init_usart_interrupt();
	//init_servo();
	//init_mot(&mot_gauche, &mot_droit);

	xTaskCreate(test, (const signed char *)"TEST", 400, NULL, 1, NULL );
	//xTaskCreate(move_zqsd, (const signed char *)"MOVE", 400, NULL, 1, NULL );
	xTaskCreate(blink_led, (const signed char*)"BLINK_LED",350,NULL,1,NULL);
	vTaskStartScheduler();

	for(;;) {

	}

	return 0;
}

void blink_led()
{
	for(;;)
	{
		vTaskDelay(50);
		platform_toggle_led(PLATFORM_LED0);
	}
}

void test() 
{
	test_lidar();
	while(1);
//	move_zqsd();
/*	init_turbine();
	vTaskDelay(2000);
	platform_reset_GPIO(GPIO1);
	platform_toggle_led(PLATFORM_LED4);
	vTaskDelay(2000);
	platform_set_GPIO(GPIO1);
	*/
	
	/*while(1)
		;
*/	
	asserv_tempo();
	char cool;
	uint8_t angle=40;
	while(1)
	{
		cool=getchar();
		if (cool=='+')
			angle++;
		else 
			angle--;
		//scanf("angle :%d", &cool);
		move_servo_from_servo_module(SERVO_CANON_BAS, angle);
	}
	while(1);
	uint8_t value;
	platform_gpio_init(GPIO9, GPIO_OType_PP, GPIO_Mode_IN, GPIO_Speed_50MHz, GPIO_PuPd_NOPULL);
	while(1){
		value=platform_GPIO_get_value(GPIO9);
		printf("value: %d\r\n",value);
		if(value)
			platform_set_led(PLATFORM_LED3);
		else
			platform_reset_led(PLATFORM_LED3);
	}
	while(1){
		ausbeeSetAngleServo(&servo1, 10);
		vTaskDelay(150);
		ausbeeSetAngleServo(&servo1, 50);
		vTaskDelay(150);
	}
	//test_canon();
	while(1);
}


void asserv_tempo()
{
	move_servo_from_servo_module(SERVO_CANON_BAS,79);
	move_servo_from_servo_module(SERVO_CANON_HAUT,24);
	move_servo_from_servo_module(SERVO_PEINTURE_COTE_AUSBEE,52);
	move_servo_from_servo_module(SERVO_PEINTURE_CANON,40);
	move_servo_from_servo_module(SERVO_BRAS_GAUCHE,12);
	move_servo_from_servo_module(SERVO_BRAS_DROITE,91);
	platform_gpio_init(GPIO9, GPIO_OType_PP, GPIO_Mode_IN, GPIO_Speed_50MHz, GPIO_PuPd_NOPULL);
	platform_gpio_init(GPIO8, GPIO_OType_PP, GPIO_Mode_IN, GPIO_Speed_50MHz, GPIO_PuPd_DOWN);
	while(platform_GPIO_get_value(GPIO8))
		;	
	platform_reset_GPIO(GPIO1);
	ausbee_l298_set_duty_cycle(mot_gauche,60);
	ausbee_l298_set_duty_cycle(mot_droit,60);
	vTaskDelay(80);
	ausbee_l298_invert_output(mot_gauche,0);
	ausbee_l298_invert_output(mot_droit,1);
	vTaskDelay(75);
	ausbee_l298_set_duty_cycle(mot_gauche,0);
	ausbee_l298_set_duty_cycle(mot_droit,0);
	move_servo_from_servo_module(SERVO_BRAS_GAUCHE,70);
	vTaskDelay(50);
	ausbee_l298_invert_output(mot_gauche,1);
	ausbee_l298_invert_output(mot_droit,0);
	ausbee_l298_set_duty_cycle(mot_gauche,60);
	ausbee_l298_set_duty_cycle(mot_droit,60);
	vTaskDelay(75);
	ausbee_l298_invert_output(mot_gauche,0);
	ausbee_l298_invert_output(mot_droit,0);
	vTaskDelay(160);
	ausbee_l298_invert_output(mot_gauche,0);
	ausbee_l298_invert_output(mot_droit,1);
	vTaskDelay(75);
	ausbee_l298_invert_output(mot_gauche,0);
	ausbee_l298_invert_output(mot_droit,0);
	vTaskDelay(190);
	move_servo_from_servo_module(SERVO_BRAS_GAUCHE,12);
	ausbee_l298_invert_output(mot_gauche,0);
	ausbee_l298_invert_output(mot_droit,1);
	vTaskDelay(70);
	ausbee_l298_invert_output(mot_gauche,0);
	ausbee_l298_invert_output(mot_droit,0);
	vTaskDelay(10);
	ausbee_l298_set_duty_cycle(mot_gauche,0);
	ausbee_l298_set_duty_cycle(mot_droit,0);
	move_servo_from_servo_module(SERVO_BRAS_GAUCHE,70);
	vTaskDelay(30);
	ausbee_l298_set_duty_cycle(mot_gauche,60);
	ausbee_l298_set_duty_cycle(mot_droit,60);
	ausbee_l298_invert_output(mot_gauche,0);
	ausbee_l298_invert_output(mot_droit,1);
	vTaskDelay(75);
	ausbee_l298_invert_output(mot_gauche,0);
	ausbee_l298_invert_output(mot_droit,0);
	move_servo_from_servo_module(SERVO_BRAS_GAUCHE,12);
	vTaskDelay(300);
	ausbee_l298_invert_output(mot_gauche,1);
	ausbee_l298_invert_output(mot_droit,0);
	vTaskDelay(220);			//75
	ausbee_l298_set_duty_cycle(mot_gauche,0);  //
	ausbee_l298_set_duty_cycle(mot_droit,0);  //
	move_servo_from_servo_module(SERVO_CANON_BAS, 50);
	vTaskDelay(60);
	move_servo_from_servo_module(SERVO_CANON_BAS,79);
	vTaskDelay(60);
	move_servo_from_servo_module(SERVO_CANON_HAUT,62);
	vTaskDelay(60);
	move_servo_from_servo_module(SERVO_CANON_HAUT,24);
	vTaskDelay(60);
	ausbee_l298_set_duty_cycle(mot_gauche,60);
	ausbee_l298_set_duty_cycle(mot_droit,60);
	vTaskDelay(10);
	ausbee_l298_set_duty_cycle(mot_gauche,0);  //
	ausbee_l298_set_duty_cycle(mot_droit,0);  //
	move_servo_from_servo_module(SERVO_CANON_BAS, 50);
	vTaskDelay(60);
	move_servo_from_servo_module(SERVO_CANON_BAS,79);
	vTaskDelay(60);
	move_servo_from_servo_module(SERVO_CANON_HAUT,62);
	vTaskDelay(60);
	move_servo_from_servo_module(SERVO_CANON_HAUT,24);
	vTaskDelay(60);
	platform_set_GPIO(GPIO1);
	ausbee_l298_invert_output(mot_gauche,0);
	ausbee_l298_invert_output(mot_droit,1);
	ausbee_l298_set_duty_cycle(mot_gauche,60);
	ausbee_l298_set_duty_cycle(mot_droit,60);
	vTaskDelay(155);	
	ausbee_l298_invert_output(mot_gauche,0);
	ausbee_l298_invert_output(mot_droit,0);
	vTaskDelay(450);
	ausbee_l298_invert_output(mot_gauche,1);
	ausbee_l298_invert_output(mot_droit,0);
	vTaskDelay(145);
	ausbee_l298_invert_output(mot_gauche,1);
	ausbee_l298_invert_output(mot_droit,1);
	while(platform_GPIO_get_value(GPIO9)!=1)
		;
	ausbee_l298_set_duty_cycle(mot_gauche,0);
	ausbee_l298_set_duty_cycle(mot_droit,0);
	move_servo_from_servo_module(SERVO_PEINTURE_COTE_AUSBEE,4);
	move_servo_from_servo_module(SERVO_PEINTURE_CANON,85);
	ausbee_l298_invert_output(mot_gauche,0);
	ausbee_l298_invert_output(mot_droit,0);
	vTaskDelay(200);
	ausbee_l298_set_duty_cycle(mot_gauche,60);
	ausbee_l298_set_duty_cycle(mot_droit,60);
	vTaskDelay(200);
	ausbee_l298_set_duty_cycle(mot_gauche,0);
	ausbee_l298_set_duty_cycle(mot_droit,0);
	while(1);


}

void foutre_les_fresque()
{
	move_servo_from_servo_module(SERVO_BRAS_GAUCHE,12);
	move_servo_from_servo_module(SERVO_BRAS_DROITE,91);
	move_servo_from_servo_module(SERVO_PEINTURE_COTE_AUSBEE,52);
	move_servo_from_servo_module(SERVO_PEINTURE_CANON,40);

	platform_gpio_init(GPIO9, GPIO_OType_PP, GPIO_Mode_IN, GPIO_Speed_50MHz, GPIO_PuPd_NOPULL);
	platform_gpio_init(GPIO8, GPIO_OType_PP, GPIO_Mode_IN, GPIO_Speed_50MHz, GPIO_PuPd_DOWN);
	while(platform_GPIO_get_value(GPIO8))
		;	
	ausbee_l298_set_duty_cycle(mot_gauche,60);
	ausbee_l298_set_duty_cycle(mot_droit,60);
	vTaskDelay(166);
	ausbee_l298_invert_output(mot_gauche,0);
	ausbee_l298_invert_output(mot_droit,1);
	vTaskDelay(155);
	ausbee_l298_invert_output(mot_gauche,0);
	ausbee_l298_invert_output(mot_droit,0);
	vTaskDelay(830);
	ausbee_l298_invert_output(mot_gauche,1);
	ausbee_l298_invert_output(mot_droit,0);
	vTaskDelay(155);
	ausbee_l298_invert_output(mot_gauche,1);
	ausbee_l298_invert_output(mot_droit,1);
	while(platform_GPIO_get_value(GPIO9)!=1)
		;
	ausbee_l298_set_duty_cycle(mot_gauche,0);
	ausbee_l298_set_duty_cycle(mot_droit,0);
	move_servo_from_servo_module(SERVO_PEINTURE_COTE_AUSBEE,4);
	move_servo_from_servo_module(SERVO_PEINTURE_CANON,85);
	ausbee_l298_invert_output(mot_gauche,0);
	ausbee_l298_invert_output(mot_droit,0);
	vTaskDelay(200);
	ausbee_l298_set_duty_cycle(mot_gauche,60);
	ausbee_l298_set_duty_cycle(mot_droit,60);
	vTaskDelay(200);
	ausbee_l298_set_duty_cycle(mot_gauche,0);
	ausbee_l298_set_duty_cycle(mot_droit,0);
	
	while(1);
}

void test_lidar()
{
	USART1ReceiveHandle=xSemaphoreCreateMutex();
	if(USART1ReceiveHandle== NULL)
	{
		platform_toggle_led(PLATFORM_LED6);
		while(1)
			;
	}
	while(1)
	{
		if(xSemaphoreTake(USART1ReceiveHandle,portMAX_DELAY)==pdTRUE)
		{
			//if(new_data){
			//	new_data=0;
			//printf("index: %X \r\n", buffer[1]);
			ausbee_lidar_parse_piccolo(buffer,data);
			int i=0;
			for(i=0; i<4; i++)
			{
				if ((data[i].distance_mm>270) && (data[i].distance_mm<330))
				{
					printf("angle:%d distance:%d \r\n", data[i].angle, data[i].distance_mm);
				}
			}
			//printf("angle1:%d \r\n", data[0].angle);
			platform_toggle_led(PLATFORM_LED3);
			
		}
	}
}

void move_zqsd()
{
	char get_clavier;
	for(;;) {
		get_clavier=getchar();
		printf("cara: %c\r\n", get_clavier);
		switch (get_clavier)
		{
			case 'z':
				ausbee_l298_invert_output(mot_gauche,0);
				ausbee_l298_invert_output(mot_droit,0);
				break;
			case 's':
				ausbee_l298_invert_output(mot_gauche,1);
				ausbee_l298_invert_output(mot_droit,1);
				break;
			case 'd':
				ausbee_l298_invert_output(mot_gauche,0);
				ausbee_l298_invert_output(mot_droit,1);
				break;
			case 'q':
				ausbee_l298_invert_output(mot_gauche,1);
				ausbee_l298_invert_output(mot_droit,0);
				break;
		/*	case 'p':
				enable_turbine=1;
				break;*/
		}
		ausbee_l298_set_duty_cycle(mot_gauche,60);
		ausbee_l298_set_duty_cycle(mot_droit,60);
		//printf("super\r\n");
		vTaskDelay(200);
		ausbee_l298_set_duty_cycle(mot_gauche,0);
		ausbee_l298_set_duty_cycle(mot_droit,0);
		platform_toggle_led(PLATFORM_LED6);
	}
}



void test_canon()
{
	ausbeeSetAngleServo(&servo2, 25);
	ausbeeSetAngleServo(&servo3, 80);
	init_turbine();
	platform_toggle_led(PLATFORM_LED6);
	vTaskDelay(1500);
	while(1)
	{
		ausbeeSetAngleServo(&servo2, 60);
		vTaskDelay(50);	
		ausbeeSetAngleServo(&servo2, 25);
		vTaskDelay(50);	
		ausbeeSetAngleServo(&servo3, 50);
		vTaskDelay(50);	
		ausbeeSetAngleServo(&servo3, 80);
		vTaskDelay(150);
	}
	while(1);
		
}

void move_servo_from_servo_module(uint8_t servo, uint8_t angle)
{
	angle > 100 ? 100 : angle ;
	printf("angle :%d \r\n", angle);
	CanTxMsg CAN_Tx;
	uint8_t mailbox_number = 0;
	if(servo & SERVO_FROM_MODULE_0)
	{
		CAN_Tx.StdId = 0x80;
		CAN_Tx.Data[0] = angle;
	}

	if(servo & SERVO_FROM_MODULE_1)
	{
		CAN_Tx.StdId = 0x81;
		CAN_Tx.Data[0] = angle;
	}

	if(servo & SERVO_FROM_MODULE_2)
	{
		CAN_Tx.StdId = 0x82;
		CAN_Tx.Data[0] = angle;
	}

	if(servo & SERVO_FROM_MODULE_3)
	{
		CAN_Tx.StdId = 0x83;
		CAN_Tx.Data[0] = angle;
	}

	if(servo & SERVO_FROM_MODULE_4)
	{
		CAN_Tx.StdId = 0x84;
		CAN_Tx.Data[0] = angle;
	}

	if(servo & SERVO_FROM_MODULE_5)
	{
		CAN_Tx.StdId = 0x85;
		CAN_Tx.Data[0] = angle;
	}

	if(servo & SERVO_FROM_MODULE_6)
	{
		CAN_Tx.StdId = 0x86;
		CAN_Tx.Data[0] = angle;
	}

	if(servo & SERVO_FROM_MODULE_7)
	{
		CAN_Tx.StdId = 0x87;
		CAN_Tx.Data[0] = angle;
	}

	CAN_Tx.ExtId = 0;
	CAN_Tx.IDE = CAN_Id_Standard;
	CAN_Tx.RTR = CAN_RTR_Data;
	CAN_Tx.DLC = 1;
	
	mailbox_number = CAN_Transmit(CAN1, &CAN_Tx);
	if(mailbox_number == CAN_TxStatus_NoMailBox)
		platform_set_led(PLATFORM_LED3);
	uint8_t transmit_status = CAN_TransmitStatus(CAN1, mailbox_number); 
	while(transmit_status!=CAN_TxStatus_Ok){
		transmit_status = CAN_TransmitStatus(CAN1, mailbox_number);
		if(transmit_status == CAN_TxStatus_Ok)
		{
			platform_set_led(PLATFORM_LED4);
			platform_reset_led(PLATFORM_LED7);
		}
		else if( transmit_status == CAN_TxStatus_Pending)
		{
			platform_set_led(PLATFORM_LED7);
		}
		else
			platform_set_led(PLATFORM_LED6);
	}

	//vTaskDelay(100);
}

void init_servo()
{
	platform_initPWM(TIMER10);
	//platform_initPWM(TIMERALL);
	ausbeeInitStructServo(& servo1);
	servo1.TIMx = SERVO1_TIM;
	servo1.CHANx = SERVO1_CHAN;
	ausbeeInitServo(&servo1);
/*	
	ausbeeInitStructServo(& servo2);
	servo2.TIMx = SERVO2_TIM;
	servo2.CHANx = SERVO2_CHAN;
	ausbeeInitServo(&servo2);
	
	ausbeeInitStructServo(& servo3);
	servo3.TIMx = SERVO3_TIM;
	servo3.CHANx = SERVO3_CHAN;
	ausbeeInitServo(&servo3);
	
	ausbeeInitStructServo(& servo4);
	servo4.TIMx = SERVO4_TIM;
	servo4.CHANx = SERVO4_CHAN;
	ausbeeInitServo(&servo4);
*/
}

