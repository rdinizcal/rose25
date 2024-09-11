#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

#include "monitor_types.h"
#include "monitor.h"

static int64_t input_signal_cpy;
static float input_signal_float_cpy;
static double input_signal_double_cpy;

static bool handlerTestCopilot_guard(void) {
  printf("Hello, World! Monitor1");
  return !(((input_signal_cpy) >= ((int64_t)(5))) && (((input_signal_float_cpy) >= ((float)(5.0f))) && ((input_signal_double_cpy) >= ((double)(5.0)))));
}

void step(void) {
  (input_signal_cpy) = (input_signal);
  (input_signal_float_cpy) = (input_signal_float);
  (input_signal_double_cpy) = (input_signal_double);
  if ((handlerTestCopilot_guard)()) {
    {(handlerTestCopilot)();}
  };
}
