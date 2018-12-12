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

bool mpu_new_data_available(void);

uint8_t mpu_who_am_i(void);

uint8_t mpu_who_am_i_ak8963(void);

void mpu_self_test(void);

void mpu_calibrate(void);

void mpu_init(void); //Base initialization

void mpu_init_kj(void); //Custom initialization

void mpu_init_ak8963(void);

#endif // _MPU9250_H_
