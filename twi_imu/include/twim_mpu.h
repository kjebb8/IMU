#ifndef _TWIM_MPU_H_
#define _TWIM_MPU_H_

#include "nrfx_twim.h"
#include "config.h"

/* TWIM instance ID. */
#define TWIM_INSTANCE_ID 0

void twim_mpu_init (void);
void twim_mpu_uninit (void);

void write_register_byte(uint8_t slave_address,
                         uint8_t register_address,
                         uint8_t data);

void write_register_twim(uint8_t   slave_address,
                         uint8_t   register_address,
                         uint8_t * p_data,
                         size_t    length);

void read_register_twim(uint8_t   slave_address,
                        uint8_t   register_address,
                        uint8_t * p_data,
                        size_t    length);

//WARNING only block with this after a TWIM transfer WITH CALLBACK
void wait_for_xfer(void);

#endif // _TWIM_MPU_H_
