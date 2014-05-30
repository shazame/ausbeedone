#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "platform.h"

#include <AUSBEE/lidar.h>

#include "define.h"

#include "init.h"

#include "lidar_detect.h"

volatile struct ausbee_lidar_data data[AUSBEE_LIDAR_PICCOLO_DATA_LENGTH];
extern volatile unsigned char buffer[AUSBEE_LIDAR_PICCOLO_FRAME_LENGTH];

extern xSemaphoreHandle USART1ReceiveHandle;

static uint8_t lidar_detect_enabled = 1;

void lidar_detect_task(void *data);

void lidar_detect_start(struct trajectory_manager *t)
{
  init_lidar();
  xTaskCreate(lidar_detect_task, (const signed char *)"LidarDetect", 200, (void *)t, 1, NULL );
}

void lidar_detect_task(void *data)
{
  struct trajectory_manager *t = (struct trajectory_manager *)data;

  for (;;) {
    // When an obstacle is detected we pause and turn on a led
    while(!lidar_detect_obstacle());
    trajectory_pause(t);
    platform_led_set(PLATFORM_LED6);

    // Then if no obstacle is detected for 0.5 s, we resume the trajectory and turn off the led
    TIM_SetCounter(TIM7,0);
    while(TIM_GetCounter(TIM7)<8400) // 0.5s
    {
      if (lidar_detect_obstacle()) {
        TIM_SetCounter(TIM7,0);
        platform_led_reset(PLATFORM_LED1);
      }
    }
    trajectory_resume(t);
    platform_led_reset(PLATFORM_LED6);
    platform_led_reset(PLATFORM_LED1);
  }
}

//Function used to use the Lidar as a GP2
//detect all things in a circle in front of the robot
//Circle parameters:
//    --> Diameter: 36 cm
//    --> center is 10.5cm far from the lidar, aligned with it.
uint8_t lidar_detect_obstacle(void)
{
  if (!lidar_detect_enabled)
    return 0;

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

        // Get point coordinates

        x = (double)data[i].distance_mm*cos((float)(angleRad))-(double)DISTANCE_CENTER_TO_LIDAR;
        y = (double)data[i].distance_mm*sin((float)(angleRad));
        if(sqrt(x*x+y*y)<(double)(CIRCLE_LIDAR_DIAMETER/2))
        {
          //printf("obstacle detecté position: x: %lf y: %lf \r\n",x,y);
          //printf("angle: %d distance: %d \r\n", data[i].angle, data[i].distance_mm);
          return 1;
        }
      }
    }
  }
  return 0;
}

void lidar_detect_enable(void)
{
  lidar_detect_enabled = 1;
}

void lidar_detect_disable(void)
{
  lidar_detect_enabled = 0;
}
