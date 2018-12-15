#ifndef _APP_TIME_KEEPER_H_
#define _APP_TIME_KEEPER_H_

#include <stdint.h>

void init_time_keeper(uint16_t freq);

float update_time(void);

#endif // _APP_TIME_KEEPER_H_
