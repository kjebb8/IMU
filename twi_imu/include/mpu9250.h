#ifndef _MPU9250_H_
#define _MPU9250_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    HZ_200,
    HZ_50
} mpu_sample_rate_t;

typedef enum {
    POLLING,
    INTERRUPT
} mpu_data_ready_t;

typedef enum {
    ACCEL,
    GYRO,
    MAG
} mpu_data_type_t;

typedef struct
{
    mpu_sample_rate_t sample_rate;
    mpu_data_ready_t  data_notification;
} mpu_init_t;

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

void mpu_init(const mpu_init_t * p_params); //Base initialization

void mpu_init_ak8963(void);

bool mpu_new_data_int(void);

bool mpu_new_data_poll(void); //Use if POLLING mode

const mpu_data_t * mpu_read_accel_data(void);

const mpu_data_t * mpu_read_gyro_data(void);

const mpu_data_t * mpu_read_mag_data(void);

#endif // _MPU9250_H_
