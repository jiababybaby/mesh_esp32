#include "mdf_common.h"
#include "mwifi.h"
#include "driver/uart.h"
#include "mesh.h"
#include "math.h"
static const char *TAG = "ZNWG-UART";
#define BUF_SIZE 200
float c0_value=0;
int pm_1_0_value=0, pm_2_5_value=0, pm_10_value=0;
char zp15_val[20]={0};
char pm_1_0_val[20]={0},pm_2_5_val[20]={0},pm_10_val[20]={0};
/**
 * @brief uart initialization
 */
#define zh06_uart 0
static mdf_err_t uart_initialize()
{
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    MDF_ERROR_ASSERT(uart_param_config(zh06_uart, &uart_config));
    MDF_ERROR_ASSERT(uart_set_pin(zh06_uart, 25, 26,0,0));
    MDF_ERROR_ASSERT(uart_driver_install(zh06_uart, 2 * BUF_SIZE, 2 * BUF_SIZE, 0, NULL, 0));
    return MDF_OK;
}
#define ZE15_UART 1
static mdf_err_t uart1_initialize()
{
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    MDF_ERROR_ASSERT(uart_param_config(ZE15_UART, &uart_config));
    MDF_ERROR_ASSERT(uart_set_pin(ZE15_UART, 33, 32,0,0));
    MDF_ERROR_ASSERT(uart_driver_install(ZE15_UART, 2 * BUF_SIZE, 2 * BUF_SIZE, 0, NULL, 0));
    return MDF_OK;
}
unsigned char FucCheckSum_co(unsigned char *i,unsigned char ln)
{
    unsigned char j,tempq=0;
    i+=1;
    for(j=0;j<(ln-2);j++)
    {
        tempq+=*i;
        i++;
    } 
    tempq=(~tempq)+1;
    return(tempq);
}
unsigned short FucCheckSum_pm2_5(unsigned char *i,unsigned char ln)
{
    unsigned char j=0;
    unsigned short tempq=0;
    for(j=0;j<(ln-2);j++)
    {
        tempq+=*i;
        i++;
    } 
    return(tempq);
}
void uart_handle_task(void *arg)
{
    int recv_length   = 0;
    mdf_err_t ret     = MDF_OK;
    cJSON *json_MCHL  = NULL;
    cJSON *json_ssid  = NULL;
    cJSON *json_pwd  = NULL;
    cJSON *json_mode  = NULL;
    cJSON *json_root  = NULL;
    cJSON *json_addr  = NULL;
    cJSON *json_group = NULL;
    cJSON *json_data  = NULL;
    cJSON *json_dest_addr  = NULL;

    // Configure a temporary buffer for the incoming data
    uint8_t *data                     = (uint8_t *) MDF_MALLOC(BUF_SIZE);
    size_t size                       = MWIFI_PAYLOAD_LEN;
    char *jsonstring                  = NULL;
    uint8_t dest_addr[MWIFI_ADDR_LEN] = {0};
    mwifi_data_type_t data_type       = {0};
    uint8_t sta_mac[MWIFI_ADDR_LEN]   = {0};

    MDF_LOGI("Uart handle task is running");

    esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);

    /* uart initialization */
    MDF_ERROR_ASSERT(uart_initialize());
    uint8_t config_flag=0;
    while (1) {
        uint8_t rec_config=0;
        memset(data, 0, BUF_SIZE);
        recv_length = uart_read_bytes(zh06_uart, data, BUF_SIZE, 100 / portTICK_PERIOD_MS);
        if (recv_length <= 0) {
            continue;
        }

        //MDF_LOGI("Uart receice %s length %d",data,recv_length);
        if(recv_length>10){
            MDF_LOGI("UART Recv data: %x %x", data[recv_length-1],FucCheckSum_pm2_5(data,recv_length));
            if(data[0]==0x42&&data[1]==0x4D&&(data[recv_length-2]<<8|data[recv_length-1])==FucCheckSum_pm2_5(data,recv_length)){
                pm_1_0_value=((data[10]<<8)|data[11]);
                pm_2_5_value=((data[12]<<8)|data[13]);
                pm_10_value=((data[14]<<8)|data[15]);
                MDF_LOGI("Uart receice pm_1_0_value %d",pm_1_0_value);
                MDF_LOGI("Uart receice pm_2_5_value %d",pm_2_5_value);
                MDF_LOGI("Uart receice pm_10_value %d",pm_10_value);
                memset(pm_1_0_val,0,sizeof(pm_1_0_val));
                memset(pm_2_5_val,0,sizeof(pm_2_5_val));
                memset(pm_10_val,0,sizeof(pm_10_val));
                sprintf(pm_1_0_val,"%d",pm_1_0_value);
                sprintf(pm_2_5_val,"%d",pm_2_5_value);
                sprintf(pm_10_val,"%d",pm_10_value);
            }
            char str_temp[200]={0};
            int i=0;
            for(i=0;i<recv_length;i++){
                char char_temp[5]={0};
                sprintf(char_temp,"%x ",data[i]);
                strcat(str_temp,char_temp);
            }
            MDF_LOGI("UART Recv data: %s", str_temp);
        }

    }

    MDF_LOGI("Uart handle task is exit");

    MDF_FREE(data);
    vTaskDelete(NULL);
}
void uart1_handle_task(void *arg)
{
    int recv_length   = 0;
    mdf_err_t ret     = MDF_OK;

    // Configure a temporary buffer for the incoming data
    uint8_t *data                     = (uint8_t *) MDF_MALLOC(BUF_SIZE);
    size_t size                       = MWIFI_PAYLOAD_LEN;

    MDF_LOGI("Uart1 handle task is running");

    /* uart1 initialization */
    MDF_ERROR_ASSERT(uart1_initialize());
    uint8_t config_flag=0;
    while (1) {
        uint8_t rec_config=0;
        memset(data, 0, BUF_SIZE);
        recv_length = uart_read_bytes(ZE15_UART, data, BUF_SIZE, 100 / portTICK_PERIOD_MS);
        if (recv_length <= 0) {
            continue;
        }
        MDF_LOGI("Uart1 receice %s length %d",data,recv_length);
        MDF_LOGI("UART1 Recv data: %x %x", data[recv_length-1],FucCheckSum_co(data,recv_length));
        if(data[0]==0xff&&data[1]==0x04&&data[recv_length-1]==FucCheckSum_co(data,recv_length)){
            c0_value=(float)(((data[4]&0x1f)<<8)|data[5])/pow(10,data[3]);
            MDF_LOGI("UART1 Recv data: %f", c0_value);
            sprintf(zp15_val,"%f",c0_value);
        }
        char str_temp[50]={0};
        int i=0;
        for(i=0;i<recv_length;i++){
            char char_temp[5]={0};
            sprintf(char_temp,"%x ",data[i]);
            strcat(str_temp,char_temp);
        }
        MDF_LOGI("UART1 Recv data: %s", str_temp);
    }

    MDF_LOGI("Uart1 handle task is exit");

    MDF_FREE(data);
    vTaskDelete(NULL);
}