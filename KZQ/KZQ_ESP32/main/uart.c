#include "mdf_common.h"
#include "mwifi.h"
#include "driver/uart.h"
#include "mesh.h"
static const char *TAG = "ZNWG-UART";
#define BUF_SIZE 128
/**
 * @brief uart initialization
 */
mdf_err_t uart_initialize()
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    MDF_ERROR_ASSERT(uart_param_config(CONFIG_UART_PORT_NUM, &uart_config));
    MDF_ERROR_ASSERT(uart_set_pin(CONFIG_UART_PORT_NUM, 17, 16,0,0));
    MDF_ERROR_ASSERT(uart_driver_install(CONFIG_UART_PORT_NUM, 2 * BUF_SIZE, 2 * BUF_SIZE, 0, NULL, 0));
    return MDF_OK;
}
