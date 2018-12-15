/**
 * Copyright (c) 2015 - 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 * @defgroup tw_sensor_example main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI Sensor Example main file.
 *
 * This file contains the source code for a sample application using TWI.
 *
 */
#include <stdio.h>

#include "twim_mpu.h"
#include "mpu9250.h"

#include "nrfx_gpiote.h"
#include "config.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

static volatile bool new_data_ready = false;

static void gpio_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    new_data_ready = true;
}

static void gpio_int_init(void)
{
    ret_code_t err_code;
    err_code = nrfx_gpiote_init();
    APP_ERROR_CHECK(err_code);

    const nrfx_gpiote_in_config_t config = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(false);
    err_code = nrfx_gpiote_in_init(INTERRUPT_PIN, &config, gpio_handler);
    APP_ERROR_CHECK(err_code);

    nrfx_gpiote_in_event_enable(INTERRUPT_PIN, true);
}

/**
 * @brief Function for main application entry.
 */
int main(void)
{
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("IMU Program Started.");
    twim_mpu_init();

    uint8_t who_am_i = mpu_who_am_i();
    NRF_LOG_INFO("MPU9250 should be 0x71 and is: 0x%02x", who_am_i);

    if(who_am_i == 0x71)
    {
        NRF_LOG_INFO("Self Test");
        mpu_self_test();
        NRF_LOG_INFO("Calibration");
        mpu_calibrate();

        NRF_LOG_INFO("GPIO Init");
        gpio_int_init();
        NRF_LOG_INFO("MPU Init");
        mpu_init();

        uint8_t who_am_i_ak = mpu_who_am_i_ak8963();
        NRF_LOG_INFO("AK8963 should be 0x48 and is: 0x%02x", who_am_i_ak);

        NRF_LOG_INFO("Mag Init");
        mpu_init_ak8963();
        NRF_LOG_INFO("Setup Successful");
        NRF_LOG_FLUSH();

        while (true)
        {
            do {
                __WFE();
            // } while(!mpu_new_data_poll());
            } while(!new_data_ready);
            new_data_ready = false;

            mpu_read_new_data();
            mpu_calculate_orientation();
            // const mpu_result_t * orientation = mpu_get_current_orientation();

            NRF_LOG_FLUSH();
        }
    }
}

/** @} */
