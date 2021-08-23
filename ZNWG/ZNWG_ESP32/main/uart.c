#include "mdf_common.h"
#include "mwifi.h"
#include "driver/uart.h"
#include "mesh.h"
#include "gatts.h"
#include "spiffs.h"
static const char *TAG = "ZNWG-UART";
#define BUF_SIZE 512
/**
 * @brief uart initialization
 */
static mdf_err_t uart_initialize()
{
    uart_config_t uart_config = {
        .baud_rate = CONFIG_UART_BAUD_RATE,
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

void uart_handle_task(void *arg)
{
    int recv_length   = 0;
    mdf_err_t ret     = MDF_OK;
    cJSON *json_mode  = NULL;
    cJSON *json_mesh_id  = NULL;
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

        if(mode_update_flag){ //配置信息更新
            char *p;
            mode_update_flag=0;
            char send_data[200]={0};
            cJSON *json_config  = NULL;
            json_config = cJSON_CreateObject();
            cJSON_AddItemToObject(json_config, "mode", cJSON_CreateNumber(mode));
            p=cJSON_Print(json_config);
            strcpy(send_data,p);
            if(!cJSON_IsInvalid(json_config)){
                cJSON_Delete(json_config);
            }
            free(p);
            uart_write_bytes(2,send_data,strlen(send_data));
            save_flash();
        }
          
        uint8_t rec_config=0;
        memset(data, 0, BUF_SIZE);
        recv_length = uart_read_bytes(CONFIG_UART_PORT_NUM, data, BUF_SIZE, 100 / portTICK_PERIOD_MS);
        if (recv_length <= 0) {
            continue;
        }
        

        ESP_LOGI("UART Recv data:", "%s", data);

        json_root = cJSON_Parse((char *)data);
        MDF_ERROR_CONTINUE(!json_root, "cJSON_Parse, data format error, data: %s", data);
        json_mode=cJSON_GetObjectItem(json_root, "mode");
        if(json_mode){
            rec_config++;
            mode=json_mode->valueint;
            if(mode==0){
                MDF_LOGW("wifi mode ");
            }
            if(mode==1){
                MDF_LOGW("eth mode ");
            }
            if(mode==2){
                MDF_LOGW("4G mode ");
            }
        }
        json_mesh_id=cJSON_GetObjectItem(json_root, "mesh_id");
        if(json_mesh_id){
             MDF_LOGW("mesh id ");
             rec_config++;
             strcpy(mesh_id,json_mesh_id->valuestring);
             ESP_LOGI("mesh_id data:", "%s", mesh_id);
        }
        if(rec_config==2){
            char send_data[200]={0};
            cJSON *json_config  = NULL;
            json_config = cJSON_CreateObject();
            cJSON_AddItemToObject(json_config, "config_ans", cJSON_CreateNumber(1));
            char *p=cJSON_Print(json_config);
            strcpy(send_data,p);
            if(!cJSON_IsInvalid(json_config)){
                cJSON_Delete(json_config);
            }
            free(p);
            uart_write_bytes(2,send_data,strlen(send_data));
            ESP_LOGI("send data:", "%s", send_data);
        }
        if(rec_config==2&&!config_flag){
            
            rec_config=0;
            config_flag=1;      
            wifi_mesh_init();
        }
        if(mode==0){
            continue;
        }
        /**
         * @brief Check if it is a group address. If it is a group address, data_type.group = true.
         */
        json_addr = cJSON_GetObjectItem(json_root, "device_mac");
        json_group = cJSON_GetObjectItem(json_root, "group");

        if (json_addr) {
            data_type.group = false;
            json_dest_addr = json_addr;
        } else if (json_group) {
            data_type.group = true;
            json_dest_addr = json_group;
        } else {
            MDF_LOGW("Address not found");
            cJSON_Delete(json_root);
            continue;
        }

        /**
         * @brief  Convert mac from string format to binary
         */
        do {
            uint32_t mac_data[MWIFI_ADDR_LEN] = {0};
            sscanf(json_dest_addr->valuestring, MACSTR,
                   mac_data, mac_data + 1, mac_data + 2,
                   mac_data + 3, mac_data + 4, mac_data + 5);

            for (int i = 0; i < MWIFI_ADDR_LEN; i++) {
                dest_addr[i] = mac_data[i];
            }
        } while (0);
        cJSON *json_device_id  = NULL;
        cJSON *json_device_data  = NULL;
        cJSON *json_data_method  = NULL;
        cJSON *json_data_num  = NULL;
        cJSON *json_data  = cJSON_CreateObject();
        json_device_id = cJSON_GetObjectItem(json_root, "device_id");
        json_device_data = cJSON_GetObjectItem(json_root, "device_data");
        json_data_method= cJSON_GetObjectItem(json_device_data, "data_method");
        json_data_num= cJSON_GetObjectItem(json_device_data, "data_num");
        cJSON_AddItemToObject(json_data, "device_id", cJSON_CreateString(json_device_id->valuestring));
        cJSON_AddItemToObject(json_data, "data_method", cJSON_CreateString(json_data_method->valuestring));
        cJSON_AddItemToObject(json_data, "data_num", cJSON_CreateString(json_data_num->valuestring));
        char *send_data = cJSON_Print(json_data);

        ret = mwifi_write(dest_addr, &data_type, send_data, strlen(send_data), true);
        MDF_ERROR_GOTO(ret != MDF_OK, FREE_MEM, "<%s> mwifi_root_write", mdf_err_to_name(ret));

FREE_MEM:
        if(!cJSON_IsInvalid(json_data)){
            cJSON_Delete(json_data);
        }
        free(send_data);
        cJSON_Delete(json_root);
    }

    MDF_LOGI("Uart handle task is exit");

    MDF_FREE(data);
    vTaskDelete(NULL);
}