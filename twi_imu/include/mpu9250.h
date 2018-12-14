#ifndef _MPU9250_H_
#define _MPU9250_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    float yaw;
    float pitch;
    float roll;
} mpu_result_t;

typedef enum {
  ACCEL,
  GYRO,
  MAG
} mpu_data_type_t;

typedef struct
{
    mpu_data_type_t type;
    float           x;
    float           y;
    float           z;
} mpu_data_t;

uint8_t mpu_who_am_i(void);

uint8_t mpu_who_am_i_ak8963(void);

void mpu_self_test(void);

void mpu_calibrate(void);

void mpu_init(void); //Base initialization

void mpu_init_kj(void); //Custom initialization

void mpu_init_ak8963(void);

bool mpu_new_data_int(void);

bool mpu_new_data_poll(void);

const mpu_data_t * mpu_read_accel_data(void);

const mpu_data_t * mpu_read_gyro_data(void);

const mpu_data_t * mpu_read_mag_data(void);

void mpu_read_new_data(void);

void mpu_calculate_orientation(void);

const mpu_result_t * mpu_get_current_orientation(void);

#endif // _MPU9250_H_
