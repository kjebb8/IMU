#include "twim_mpu.h"
#include "config.h"

#include "nrf_delay.h"
#include "nrf_log.h"

/* TWIM instance. */
static const nrfx_twim_t m_twim = NRFX_TWIM_INSTANCE(TWIM_INSTANCE_ID);

/* Buffer for register address */
static uint8_t m_register_address;

/* Buffer for writing single byte */
static uint8_t m_data_buffer[2];

/* Indicates if operation on TWIM has ended. */
static volatile bool m_twim_xfer_in_progress = false;

static void wait_for_xfer(void)
{
    while(m_twim_xfer_in_progress);
}

/**
 * @brief TWI events handler.
 */
static void twim_handler(nrfx_twim_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRFX_TWIM_EVT_DONE:
            m_twim_xfer_in_progress = false;
            break;
        default:
            break;
    }
}

/**
 * @brief TWIM initialization.
 */
void twim_mpu_init (void)
{
    ret_code_t err_code;
    const nrfx_twim_config_t twim_config = {
        .scl                 = TWI_SCL_PIN,
        .sda                 = TWI_SDA_PIN,
        .frequency           = (nrf_twim_frequency_t) NRFX_TWIM_DEFAULT_CONFIG_FREQUENCY, //100k, same as IMU
        .interrupt_priority  = NRFX_TWIM_DEFAULT_CONFIG_IRQ_PRIORITY,
        .hold_bus_uninit     = NRFX_TWIM_DEFAULT_CONFIG_HOLD_BUS_UNINIT,
    };
    err_code = nrfx_twim_init(&m_twim, &twim_config, twim_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrfx_twim_enable(&m_twim);
}

void twim_mpu_uninit (void)
{
    nrfx_twim_disable(&m_twim);
    nrfx_twim_uninit(&m_twim);
}

void twim_mpu_write_register_byte(uint8_t slave_address, uint8_t register_address, uint8_t data)
{
    ret_code_t err_code;
    m_data_buffer[0] = register_address;
    m_data_buffer[1] = data;
    const nrfx_twim_xfer_desc_t xfer_desc  = {
        .type             = NRFX_TWIM_XFER_TX,
        .address          = slave_address,
        .primary_length   = sizeof(m_data_buffer),
        .p_primary_buf    = m_data_buffer
    };
    while(nrfx_twim_is_busy(&m_twim));
    m_twim_xfer_in_progress = true;
    err_code = nrfx_twim_xfer(&m_twim, &xfer_desc, 0);
    APP_ERROR_CHECK(err_code);
    wait_for_xfer();
}

void twim_mpu_read_register(uint8_t slave_address, uint8_t register_address, uint8_t * p_data, size_t length)
{
    ret_code_t err_code;
    m_register_address = register_address; //If transfer is asynchronous, store the value off the stack
    const nrfx_twim_xfer_desc_t xfer_desc  = {
        .type             = NRFX_TWIM_XFER_TXRX,
        .address          = slave_address,
        .primary_length   = sizeof(m_register_address),
        .secondary_length = length,
        .p_primary_buf    = &m_register_address,
        .p_secondary_buf  = p_data
    };
    while(nrfx_twim_is_busy(&m_twim));
    m_twim_xfer_in_progress = true;
    err_code = nrfx_twim_xfer(&m_twim, &xfer_desc, 0);
    APP_ERROR_CHECK(err_code);
    wait_for_xfer();
}

void twim_mpu_read_register_async(uint8_t slave_address, uint8_t register_address, uint8_t * p_data, size_t length, twim_mpu_cb_t cb)
{
    // ret_code_t err_code;
    // m_register_address = register_address; //If transfer is asynchronous, store the value off the stack
    // const nrfx_twim_xfer_desc_t xfer_desc  = {
    //     .type             = NRFX_TWIM_XFER_TXRX,
    //     .address          = slave_address,
    //     .primary_length   = sizeof(m_register_address),
    //     .secondary_length = length,
    //     .p_primary_buf    = &m_register_address,
    //     .p_secondary_buf  = p_data
    // };
    // while(nrfx_twim_is_busy(&m_twim));
    // m_twim_xfer_in_progress = true;
    // err_code = nrfx_twim_xfer(&m_twim, &xfer_desc, 0);
    // APP_ERROR_CHECK(err_code);
}
