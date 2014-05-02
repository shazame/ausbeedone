#ifndef ENCODERS_UTILS_H
#define ENCODERS_UTILS_H

#include <stdint.h>

struct encoders {
  int32_t right_value, left_value;
};

void encoders_set_right_value(int32_t);
void encoders_set_left_value (int32_t);

int32_t encoders_get_right_value(void);
int32_t encoders_get_left_value (void);

#endif /* ENCODERS_UTILS_H */
