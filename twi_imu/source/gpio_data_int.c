#include "gpio_data_int.h"
#include "nrfx_gpiote.h"

static volatile bool new_data_ready = false;

static void gpio_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    new_data_ready = true;
}

void gpio_int_init(uint32_t int_pin)
{
    ret_code_t err_code;
    err_code = nrfx_gpiote_init();
    APP_ERROR_CHECK(err_code);

    const nrfx_gpiote_in_config_t config = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(false);
    err_code = nrfx_gpiote_in_init(int_pin, &config, gpio_handler);
    APP_ERROR_CHECK(err_code);

    nrfx_gpiote_in_event_enable(int_pin, true);
}

bool gpio_new_data_ready(void)
{
    if(new_data_ready)
    {
        new_data_ready = false;
        return true;
    }
    return false;
}
