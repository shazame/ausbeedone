#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"

#include "../utils/actions.h"
#include "../utils/init.h"

#include "relay_counter.h"

#ifdef UNUSED
#elif defined(__GNUC__)
# define UNUSED(x) UNUSED_ ## x __attribute__((unused))
#elif defined(__LCLINT__)
# define UNUSED(x) /*@unused@*/ x
#else
# define UNUSED(x) x
#endif

extern volatile uint8_t elapsed_time;

void relay_counter_task(void *data);

void relay_counter_start(void)
{
  init_timer_relais();
  xTaskCreate(relay_counter_task, (const signed char *)"RelayCounter", 100, NULL, 1, NULL );
}

void relay_counter_task(void *UNUSED(dummy))
{
  while(presence_tirette())
    ;
  elapsed_time=0;
  //platform_led_set(PLATFORM_LED1);
  while(1)
  {
    if (elapsed_time>=88)
    {
      // Shutdown power
      disable_power_relay();
      //platform_led_set(PLATFORM_LED6);
    }
  }
}
