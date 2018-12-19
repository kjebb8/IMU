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
#include "quaternion_filters.h"
#include "config.h"
#include "app_time_keeper.h"
#include "orientation_calculator.h"
#include "uarte_mpu.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define PRINT_VALUES
// #define UART_OUTPUT

//Math Help
#define M_PI 3.14159265358979323846
#define DEG_TO_RAD (M_PI / 180.0f)

static const orientation_result_t * current_orientation;
static const mpu_init_t mpu_params = {
    .sample_rate       = HZ_50,
    .data_notification = INTERRUPT,
    .interrupt_pin     = INTERRUPT_PIN
};

#ifdef PRINT_VALUES
// Used to control display output rate
static uint32_t samples_since_print = 0;
static float time_since_print = 0.0f;
#endif

static void print_values(void)
{
    if(time_since_print > 0.5f)
    {
        NRF_LOG_RAW_INFO("\r\nYaw: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(current_orientation->yaw.float_val));
        NRF_LOG_RAW_INFO("\tPitch: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(current_orientation->pitch.float_val));
        NRF_LOG_RAW_INFO("\tRoll: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(current_orientation->roll.float_val));
        NRF_LOG_RAW_INFO("\tRate: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(samples_since_print / time_since_print));

        samples_since_print = 0;
        time_since_print = 0;
    }
}

/**
 * @brief Function for main application entry.
 */
int main(void)
{
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    NRF_LOG_INFO("IMU Program Started.");

    NRF_LOG_INFO("TWIM Init");
    twim_mpu_init();

#ifdef UART_OUTPUT
    NRF_LOG_INFO("UARTE Init");
    uarte_mpu_init();
#endif

    uint8_t who_am_i = mpu_who_am_i();
    NRF_LOG_INFO("MPU9250 should be 0x71 and is: 0x%02x", who_am_i);

    if(who_am_i == 0x71)
    {
        NRF_LOG_INFO("Self Test");
        mpu_self_test();
        NRF_LOG_INFO("Calibration");
        mpu_calibrate();

        NRF_LOG_INFO("MPU Init");
        mpu_init(&mpu_params);

        NRF_LOG_INFO("Time Keeper Init");
        if(mpu_params.sample_rate == HZ_200)
        {
            init_time_keeper(800);
        }
        else if(mpu_params.sample_rate == HZ_50)
        {
            init_time_keeper(200);
        }

        uint8_t who_am_i_ak = mpu_who_am_i_ak8963();
        NRF_LOG_INFO("AK8963 should be 0x48 and is: 0x%02x", who_am_i_ak);

        NRF_LOG_INFO("Mag Init");
        mpu_init_ak8963();
        NRF_LOG_INFO("Setup Successful");
        NRF_LOG_FLUSH();

        //MAIN LOOP
        while (true)
        {
            bool exit_condition;
            do {
                __WFE();

                if(mpu_params.data_notification == POLLING)
                {
                    exit_condition = mpu_new_data_poll();
                }
                else if(mpu_params.data_notification == INTERRUPT)
                {
                    exit_condition = mpu_new_data_int();
                }

            } while(!exit_condition);

            const mpu_data_t * accel_data = mpu_read_accel_data();
            const mpu_data_t * gyro_data = mpu_read_gyro_data();
            const mpu_data_t * mag_data = mpu_read_mag_data();

            float deltat = update_time();
        #ifdef PRINT_VALUES
            time_since_print += deltat;
            samples_since_print++;
        #endif

            mahony_quaternion_update(accel_data->x, accel_data->y, accel_data->z,
                                     gyro_data->x * DEG_TO_RAD, gyro_data->y * DEG_TO_RAD, gyro_data->z * DEG_TO_RAD,
                                     mag_data->y, mag_data->x, -mag_data->z, deltat);

            current_orientation = calculate_orientation(get_q());

        #ifdef PRINT_VALUES
            print_values();
        #endif

        #ifdef UART_OUTPUT
            uarte_mpu_tx(current_orientation->yaw.bytes, sizeof(float));
            uarte_mpu_tx(current_orientation->pitch.bytes, sizeof(float));
            uarte_mpu_tx(current_orientation->roll.bytes, sizeof(float));
        #endif

            NRF_LOG_FLUSH();
        }
    }
}

/** @} */
