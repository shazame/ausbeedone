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
#include "utils/init.h"
#include "utils/define.h"
#include "utils/actions.h"
#include "AUSBEE/servo.h"


// Private function prototypes
void turbine();
void test();
void blink_led();
void test_lidar();
void move_zqsd();
void test_canon();
void init_servo();
void asserv_tempo();
void foutre_les_fresque();
void move_servo_from_servo_module(uint8_t servo, uint8_t angle);
void lidar_detect_in_circle();
void relay_counter();

volatile struct ausbee_lidar_data data[AUSBEE_LIDAR_PICCOLO_DATA_LENGTH];
volatile unsigned char buffer[AUSBEE_LIDAR_PICCOLO_FRAME_LENGTH];
volatile uint8_t elapsed_time=0;
xSemaphoreHandle USART1ReceiveHandle;
xSemaphoreHandle CANReceiveSemaphore;
CanRxMsg CAN_RxStruct;

struct ausbee_l298_chip mot_gauche; //branché sur mot1
struct ausbee_l298_chip mot_droit;  //branché sur mot2
ausbeeServo servo1, servo2, servo3, servo4;

int main(void) {
  // Call the platform initialization function
  platform_hse_pll_init();
  platform_usart_init(USART_DEBUG, 115200);
  //platform_usart_init(USART1,115200);
  platform_led_init();
  init_timer_relais();
  //platform_can_init(CAN1);
  //init_can();
  //init_turbine();
  //init_lidar();
  //init_usart_interrupt();
  //init_servo();
  //init_mot(&mot_gauche, &mot_droit);
  init_gpio_robot();
  //init_servo_position_depart();
  //xTaskCreate(test, (const signed char *)"TEST", 400, NULL, 1, NULL );
  //xTaskCreate(move_zqsd, (const signed char *)"MOVE", 400, NULL, 1, NULL );
  xTaskCreate(blink_led, (const signed char*)"BLINK_LED",100,NULL,1,NULL);
  //xTaskCreate(turbine, (const signed char*)"TURBINE",350,NULL,1,NULL);
  xTaskCreate(relay_counter, (const signed char*)"RELAY",100,NULL,1,NULL);
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
    platform_led_toggle(PLATFORM_LED0);
  }
}

void relay_counter()
{
  while(contact_fresque()!=1)
    ;
  elapsed_time=0;
  platform_led_set(PLATFORM_LED1);
  while(1)
  {
    if (elapsed_time>=88)
    {
      //shutdown power
      disable_power_relay();
      platform_led_set(PLATFORM_LED6);
    }
  }
}


void turbine()
{
  while(1)
  {
    if(couleur_depart()==COULEUR_JAUNE)
      enable_turbine();
    else if(couleur_depart()==COULEUR_ROUGE)
      disable_turbine();
  }
}

void test()
{
  //asserv_tempo();
  //while(1)
  //{
  //  while(contact_fresque()!=1)
  //    ;
  //  lancer_une_balle();
  //}
  //lidar_detect_in_circle();
  //test_lidar();
  //while(1);
  //move_zqsd();
  

  /*while(1)
    ;
  */
  //asserv_tempo();
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
   move_servo_from_servo_module(SERVO_BRAS_GAUCHE, angle);
 }
  //while(1);
  uint8_t value;
  platform_gpio_init(GPIO6, GPIO_OType_PP, GPIO_Mode_IN, GPIO_Speed_50MHz, GPIO_PuPd_NOPULL);
  while(1){
    value=platform_gpio_get_value(GPIO6);
    printf("value: %d\r\n",value);
    if(value)
      platform_led_set(PLATFORM_LED3);
    else
      platform_led_reset(PLATFORM_LED3);
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
  init_servo_position_depart();
  init_gpio_robot(); 
  while(presence_tirette())
    ;
  enable_turbine();
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
  vTaskDelay(220);                           //75
  ausbee_l298_set_duty_cycle(mot_gauche,0);  //
  ausbee_l298_set_duty_cycle(mot_droit,0);   //
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
  platform_gpio_set(GPIO1);
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
  while(platform_gpio_get_value(GPIO9)!=1)
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
  while(platform_gpio_get_value(GPIO8))
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
  while(platform_gpio_get_value(GPIO9)!=1)
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
  //USART1ReceiveHandle=xSemaphoreCreateMutex();
  //if(USART1ReceiveHandle== NULL)
  //{
  //  platform_led_toggle(PLATFORM_LED6);
  //}
  while(1)
  {
    if(xSemaphoreTake(USART1ReceiveHandle,100)==pdTRUE)
    {
      //if(new_data)
      //  new_data=0;
      //printf("index: %X \r\n", buffer[1]);
      ausbee_lidar_parse_piccolo(buffer,data);
      int i=0;
      for(i=0; i<4; i++)
      {
        if ((data[i].distance_mm>250) && (data[i].distance_mm<350))
        {
          printf("angle:%d distance:%d \r\n", data[i].angle, data[i].distance_mm);
        }
      }
      //printf("angle1:%d \r\n", data[0].angle);
      platform_led_toggle(PLATFORM_LED3);

      }
    }
}

void test_lidar_robot_running()
{

}

//Function used to use the Lidar as a GP2
//detect all things in a circle in front of the robot
//Circle parameters: 
//    --> Diameter: 36 cm
//    --> center is 10.5cm far from the lidar, aligned with it.
void lidar_detect_in_circle()
{
  while(1)
  {
    if(xSemaphoreTake(USART1ReceiveHandle,100)==pdTRUE)
    {
      ausbee_lidar_parse_piccolo(buffer,data);
      int i=0;
      for(i=0; i<AUSBEE_LIDAR_PICCOLO_DATA_LENGTH; i++)
      {
	      if ((!data[i].error) && (!data[i].strengthWarning)) //If data is valid
        { 
          double x, y;
          double angleRad;

          angleRad = data[i].angle * 2 * M_PI / 360.0; // Degree to radian conversion

          //x = (double)data[i].distance_mm*cos((float)(angleRad))-(double)DISTANCE_CENTER_TO_LIDAR;
          //y = (double)data[i].distance_mm*sin((float)(angleRad));
          //if (data[i].distance_mm<400)
          //{ 
          //  printf("obstacle detecté position: x: %lf y: %lf \r\n",x,y);
          //  printf("angle: %d distance: %d \r\n", data[i].angle, data[i].distance_mm);
          //}
          // Get point coordinates
          
          x = (double)data[i].distance_mm*cos((float)(angleRad))-(double)DISTANCE_CENTER_TO_LIDAR;
          y = (double)data[i].distance_mm*sin((float)(angleRad));
          if(sqrt(x*x+y*y)<(double)(CIRCLE_LIDAR_DIAMETER/2))
          {
            printf("obstacle detecté position: x: %lf y: %lf \r\n",x,y);
            printf("angle: %d distance: %d \r\n", data[i].angle, data[i].distance_mm);
          }
        }
      }
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
      /*case 'p':
        enable_turbine=1;
        break;*/
    }
    ausbee_l298_set_duty_cycle(mot_gauche,60);
    ausbee_l298_set_duty_cycle(mot_droit,60);
    //printf("super\r\n");
    vTaskDelay(200);
    ausbee_l298_set_duty_cycle(mot_gauche,0);
    ausbee_l298_set_duty_cycle(mot_droit,0);
    platform_led_toggle(PLATFORM_LED6);
  }
}

void test_canon()
{
  ausbeeSetAngleServo(&servo2, 25);
  ausbeeSetAngleServo(&servo3, 80);
  init_turbine();
  platform_led_toggle(PLATFORM_LED6);
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


void init_servo()
{
  platform_pwm_init(TIMER10);
  //platform_pwm_init(TIMERALL);
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
