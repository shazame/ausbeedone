#include "encoders.h"

struct encoders enc = { 0, 0 };

void encoders_set_right_value(int32_t val)
{
  enc.right_value = val;
}
void encoders_set_left_value (int32_t val)
{
  enc.left_value = val;
}

int32_t encoders_get_right_value(void)
{
  return enc.right_value;
}

int32_t encoders_get_left_value (void)
{
  return enc.left_value;
}
