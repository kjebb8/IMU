#ifndef _TWIM_MPU_H_
#define _TWIM_MPU_H_

#include <stdint.h>

typedef void (* twim_mpu_cb_t) (void);

void twim_mpu_init (void);
void twim_mpu_uninit (void);

void twim_mpu_write_register_byte(uint8_t slave_address,
                                  uint8_t register_address,
                                  uint8_t data);

void twim_mpu_read_register(uint8_t   slave_address,
                            uint8_t   register_address,
                            uint8_t * p_data,
                            uint8_t    length);

void twim_mpu_read_register_async(uint8_t       slave_address,
                                  uint8_t       register_address,
                                  uint8_t *     p_data,
                                  uint8_t       length,
                                  twim_mpu_cb_t cb);

#endif // _TWIM_MPU_H_
