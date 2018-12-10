#ifndef _MPU9250_H_
#define _MPU9250_H_

#include "twi_mpu.h"

#define TWI_MPU_ADDRESS    0x68
#define WHO_AM_I_MPU9250   0x75

volatile bool new_imu_data = false;

uint8_t mpu9250mpu_who_am_i();

#endif // _MPU9250_H_
