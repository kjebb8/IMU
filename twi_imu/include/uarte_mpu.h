#ifndef _UARTE_MPU_H_
#define _UARTE_MPU_H_

#include <stdint.h>

void uarte_mpu_init(void);
void uarte_mpu_tx(const uint8_t * p_data, uint8_t length);

#endif // _UARTE_MPU_H_
