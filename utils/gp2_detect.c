#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"
//#include "semphr.h"
#include "platform.h"

#include <AUSBEE/gp2.h>
#include <AUSBEE/quadramp.h>

#include "define.h"

#include "gp2_detect.h"

static uint8_t gp2_detect_enabled = 1;
static struct control_system *cs;

void gp2_detect_task(void *data);

void gp2_detect_start(struct trajectory_manager *t, struct control_system *am)
{
  cs = am;
  platform_gpio_init(PLATFORM_GPIO8_PIN, GPIO_OType_PP, GPIO_Mode_IN, GPIO_Speed_50MHz, GPIO_PuPd_DOWN);
  xTaskCreate(gp2_detect_task, (const signed char *)"GP2Detect", 200, (void *)t, 1, NULL );
}

void gp2_detect_task(void *data)
{
  struct trajectory_manager *t = (struct trajectory_manager *)data;

  for (;;) {
    // When an obstacle is detected we pause and turn on a led
    while(!gp2_detect_obstacle()) {
      //vTaskDelay(10 / portTICK_RATE_MS);
    }
    trajectory_pause(t);
    ausbee_quadramp_reset(&(cs->quadramp_distance));
    ausbee_quadramp_reset(&(cs->quadramp_angle));
    platform_led_set(PLATFORM_LED7);

    while(gp2_detect_obstacle())
    {
      //vTaskDelay(10 / portTICK_RATE_MS);
    }
    trajectory_resume(t);
    platform_led_reset(PLATFORM_LED7);

    vTaskDelay(10 / portTICK_RATE_MS);
  }
}

uint8_t gp2_detect_obstacle(void)
{
  if (!gp2_detect_enabled)
    return 0;

  if (platform_gpio_get_value(PLATFORM_GPIO8_PIN))
  {
    return 1;
  }

  return 0;
}

void gp2_detect_enable(void)
{
  gp2_detect_enabled = 1;
}

void gp2_detect_disable(void)
{
  gp2_detect_enabled = 0;
}
