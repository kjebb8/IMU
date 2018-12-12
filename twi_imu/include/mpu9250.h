#ifndef _MPU9250_H_
#define _MPU9250_H_

#include "mpu9250_support.h"
#include "twim_mpu.h"

typedef struct
{
    float yaw;
    float pitch;
    float roll;
} mpu_result_t;

void mpu9250_init(void);

bool mpu_new_data_available(void);

uint8_t mpu_who_am_i(void);

void mpu_self_test(void);

#endif // _MPU9250_H_
