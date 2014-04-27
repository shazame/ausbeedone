#ifndef ENCODERS_UTILS_H
#define ENCODERS_UTILS_H

#include <stdint.h>

struct encoders {
  int32_t right_value, left_value;
};

void set_right_encoder_value(int32_t);
void set_left_encoder_value (int32_t);

int32_t get_right_encoder_value(void);
int32_t get_left_encoder_value (void);

#endif /* ENCODERS_UTILS_H */
