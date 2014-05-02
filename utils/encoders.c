#include "encoders.h"

struct encoders enc = { 0, 0 };

void set_right_encoder_value(int32_t val)
{
  enc.right_value = val;
}
void set_left_encoder_value (int32_t val)
{
  enc.left_value = val;
}

int32_t get_right_encoder_value(void)
{
  return enc.right_value;
}

int32_t get_left_encoder_value (void)
{
  return enc.left_value;
}
