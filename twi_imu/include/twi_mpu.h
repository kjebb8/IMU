#ifndef _TWI_MPU_H_
#define _TWI_MPU_H_

//twi_mpu handles the TWI transactions

#include "nrfx_twim.h"
#include "config.h"

/* TWI instance ID. */
#define TWI_INSTANCE_ID 0

/* Indicates if operation on TWI has ended. */
volatile bool m_twi_xfer_done = false;

void twi_mpu_init (void);
void twi_mpu_uninit (void);

void write_register_twi(uint8_t   slave_address,
                        uint8_t   register_address,
                        uint8_t * p_data,
                        size_t    length);

void read_register_twi(uint8_t   slave_address,
                       uint8_t   register_address,
                       uint8_t * p_data,
                       size_t    length)



#endif // _TWI_MPU_H_
