#ifndef _GPIO_DATA_INT_H_
#define _GPIO_DATA_INT_H_

#include <stdint.h>
#include <stdbool.h>

void gpio_int_init(uint32_t int_pin);
bool gpio_new_data_ready(void);

#endif // _GPIO_DATA_INT_H_
