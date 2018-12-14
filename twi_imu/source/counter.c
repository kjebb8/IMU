/**
 * Copyright (c) 2016 - 2017, Nordic Semiconductor ASA
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

/**@cond To Make Doxygen skip documentation generation for this file.
 * @{
 */

#include "counter.h"
#include "nrfx_clock.h"
#include "nrfx_rtc.h"


/* RTC driver instance using RTC2.
 * RTC0 is used by the SoftDevice, and RTC1 by the app_timer library. */
static const nrfx_rtc_t m_rtc = NRFX_RTC_INSTANCE(2);


static void rtc_handler(nrfx_rtc_int_type_t int_type)
{
    // Likely a counter overflow.
    APP_ERROR_CHECK(0xFFFFFFFF);
}

static void lfclk_handler(nrfx_clock_evt_type_t event)
{
    //Do nothing
}

static void lfclk_start(void)
{
    ret_code_t err_code = nrfx_clock_init(lfclk_handler);
    APP_ERROR_CHECK(err_code);

    nrfx_clock_disable();

    nrfx_clock_lfclk_start();
}

void counter_init(uint16_t freq)
{
    lfclk_start();

    ret_code_t err_code;

    // Initialize the RTC instance.
    nrfx_rtc_config_t config = NRFX_RTC_DEFAULT_CONFIG;

    uint16_t pre = (uint16_t)(32768 / freq) - 1;
    config.prescaler = pre;

    err_code = nrfx_rtc_init(&m_rtc, &config, rtc_handler);
    APP_ERROR_CHECK(err_code);

    nrfx_rtc_tick_disable(&m_rtc);
}


void counter_start(void)
{
    nrfx_rtc_counter_clear(&m_rtc);

    // Power on!
    nrfx_rtc_enable(&m_rtc);
}


void counter_stop(void)
{
    nrfx_rtc_disable(&m_rtc);
}


uint32_t counter_get(void)
{
    return(nrfx_rtc_counter_get(&m_rtc));
}

/** @}
 *  @endcond
 */
