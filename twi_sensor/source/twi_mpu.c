#include "twi_mpu.h"

/* TWI instance. */
static const nrfx_twim_t m_twim = NRFX_TWIM_INSTANCE(TWI_INSTANCE_ID)

/* Buffer for register address */
static uint8_t m_register_address = 0x00;

/**
 * @brief TWI initialization.
 */
void twi_mpu_init (void)
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

    nrfx_twim_enable(&m_twi);
}

void twi_mpu_uninit (void)
{
    nrfx_twim_disable(&m_twim);
    nrfx_twim_uninit(&m_twim);
}

void write_register_twi(uint8_t slave_address, uint8_t register_address, uint8_t * p_data, size_t length)
{
    m_twi_xfer_done = false;

    ret_code_t err_code;
    m_register_address = register_address; //If transfer is asynchronous, store the value off the stack
    const nrfx_twim_xfer_desc_t xfer_desc  = {
        .type             = NRFX_TWIM_XFER_TXTX,
        .address          = slave_address,
        .primary_length   = sizeof(m_register_address),
        .secondary_length = length,
        .p_primary_buf    = &m_register_address,
        .p_secondary_buf  = p_data
    };
    while(nrfx_twim_is_busy);
    err_code = nrfx_twim_xfer(&m_twim, &xfer_desc, NULL);
    APP_ERROR_CHECK(err_code);
}

void read_register_twi(uint8_t slave_address, uint8_t register_address, uint8_t * p_data, size_t length)
{
    m_twi_xfer_done = false;

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
    while(nrfx_twim_is_busy);
    err_code = nrfx_twim_xfer(&m_twim, &xfer_desc, NULL);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief TWI events handler.
 */
static void twim_handler(nrfx_twim_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRFX_TWIM_EVT_DONE:
            m_twim_xfer_done = true;
            break;
        default:
            break;
    }
}
