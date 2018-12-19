#include "uarte_mpu.h"
#include "nrfx_uarte.h"
#include "pca10040.h"

#define UARTE_INSTANCE_ID 0

static const nrfx_uarte_t m_uarte = NRFX_UARTE_INSTANCE(UARTE_INSTANCE_ID);

// void uarte_handler(nrfx_uarte_event_t const * p_event, void * p_context)
// {
//     //Currently Implements Blocking Mode
// }

void uarte_mpu_init(void)
{
    ret_code_t err_code;
    const nrfx_uarte_config_t uarte_config = {
        .pseltxd            = TX_PIN_NUMBER,                              \
        .pselrxd            = RX_PIN_NUMBER,                              \
        .pselcts            = CTS_PIN_NUMBER,                              \
        .pselrts            = RTS_PIN_NUMBER,                              \
        .p_context          = NULL,                                                     \
        .hwfc               = (nrf_uarte_hwfc_t)NRFX_UARTE_DEFAULT_CONFIG_HWFC,         \
        .parity             = (nrf_uarte_parity_t)NRFX_UARTE_DEFAULT_CONFIG_PARITY,     \
        .baudrate           = (nrf_uarte_baudrate_t)NRFX_UARTE_DEFAULT_CONFIG_BAUDRATE, \
        .interrupt_priority = NRFX_UARTE_DEFAULT_CONFIG_IRQ_PRIORITY,                   \
    };
    //115200 baudrate
    err_code = nrfx_uarte_init(&m_uarte, &uarte_config, NULL); //Currently in blocking mode
    APP_ERROR_CHECK(err_code);
}

void uarte_mpu_tx(const uint8_t * p_data, uint8_t length)
{
    ret_code_t err_code;
    while(nrfx_uarte_tx_in_progress(&m_uarte));
    err_code = nrfx_uarte_tx(&m_uarte, p_data, length);
    APP_ERROR_CHECK(err_code);
}
