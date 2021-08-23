
#include "mdf_common.h"
#include "mwifi.h"
#include "driver/uart.h"
#include "socket.h"
#include "mconfig_chain.h"
static const char *TAG = "ZNWG-MESH";
esp_netif_t *sta_netif;
char mesh_id[10]={0};
uint8_t mode;
uint8_t mesh_channel=0;
char ssid[100]={0};
char pwd[100]={0};

/**
 * @brief printing system information
 */
static void print_system_info_timercb(void *timer)
{
    uint8_t primary                 = 0;
    wifi_second_chan_t second       = 0;
    mesh_addr_t parent_bssid        = {0};
    uint8_t sta_mac[MWIFI_ADDR_LEN] = {0};
    wifi_sta_list_t wifi_sta_list   = {0x0};

    esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
    esp_wifi_ap_get_sta_list(&wifi_sta_list);
    esp_wifi_get_channel(&primary, &second);
    esp_mesh_get_parent_bssid(&parent_bssid);

    MDF_LOGI("System information, channel: %d, layer: %d, self mac: " MACSTR ", parent bssid: " MACSTR
             ", parent rssi: %d, node num: %d, free heap: %u", primary,
             esp_mesh_get_layer(), MAC2STR(sta_mac), MAC2STR(parent_bssid.addr),
             mwifi_get_parent_rssi(), esp_mesh_get_total_node_num(), esp_get_free_heap_size());

    for (int i = 0; i < wifi_sta_list.num; i++) {
        MDF_LOGI("Child mac: " MACSTR, MAC2STR(wifi_sta_list.sta[i].mac));
    }

#ifdef MEMORY_DEBUG

    if (!heap_caps_check_integrity_all(true)) {
        MDF_LOGE("At least one heap is corrupt");
    }

    mdf_mem_print_heap();
    mdf_mem_print_record();
    mdf_mem_print_task();
#endif /**< MEMORY_DEBUG */
}
/* 定义一个按键值消息队列句柄 */
#define mesh_read_max_length 200
QueueHandle_t mesh_Read_Queue;
char mesh_read_data[mesh_read_max_length]={0};
static void node_read_task(void *arg)
{
    mdf_err_t ret                    = MDF_OK;
    size_t size                      = mesh_read_max_length;
    mwifi_data_type_t data_type      = {0x0};
    uint8_t src_addr[MWIFI_ADDR_LEN] = {0x0};
    uint8_t sta_mac[MWIFI_ADDR_LEN] = {0};
    MDF_LOGI("Node read task is running");
    esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
      /* 创建MESH检测消息队列 */
    mesh_Read_Queue = xQueueCreate((UBaseType_t )5,      /* 队列长度，这里是队列的项目数，即这个队列可以接受多少个消息*/
                             (UBaseType_t )mesh_read_max_length);     /* 队列中每个消息的长度，单位为字节 */
    char *data=mesh_read_data;
    cJSON *json_root  = NULL;
    cJSON *json_sn  = NULL;
    cJSON *json_device_mac  = NULL;
    cJSON *json_device_status  = NULL;

    cJSON *json_device_kind  = NULL;
    cJSON *json_device_id  = NULL;
    cJSON *json_device_data  = NULL;
    char sn[20]={0};
    char *p=NULL;
    sprintf(sn,MACSTR,MAC2STR(sta_mac));
    for (;;) {
        if (!mwifi_is_connected() && !(mwifi_is_started() && esp_mesh_is_root())) {
            vTaskDelay(500 / portTICK_RATE_MS);
            continue;
        }

        size = mesh_read_max_length;
        memset(data, 0, size);
        MDF_LOGI("READY TO RECV");
        /**
         * @brief Pre-allocated memory to data and size must be specified when passing in a level 1 pointer
         */
        ret = mwifi_read(src_addr, &data_type, data, &size, portMAX_DELAY);
        MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_read", mdf_err_to_name(ret));
        MDF_LOGI("Node receive, addr: " MACSTR ", size: %d, data: %s", MAC2STR(src_addr), size, data);

        json_root = cJSON_Parse((char *)data);
        json_sn =  cJSON_CreateObject();
        cJSON_AddItemToObject(json_sn, "sn", cJSON_CreateString(sn));

        json_device_mac=cJSON_GetObjectItem(json_root,"device_mac");
        json_device_status=cJSON_GetObjectItem(json_root,"device_status");
        json_device_kind=cJSON_GetObjectItem(json_root, "device_kind");
        json_device_id=cJSON_GetObjectItem(json_root, "device_id");
        json_device_data=cJSON_GetObjectItem(json_root, "device_data");   
        if(json_device_mac)
            cJSON_AddItemToObject(json_sn, "device_kind", cJSON_CreateString(json_device_kind->valuestring));
        if(json_device_id)
            cJSON_AddItemToObject(json_sn, "device_id", cJSON_CreateString(json_device_id->valuestring));
        if(json_device_data)
            cJSON_AddItemToObject(json_sn, "device_data", cJSON_CreateString(json_device_data->valuestring));   
        if(json_device_mac)
            cJSON_AddItemToObject(json_sn,"device_mac",cJSON_CreateString(json_device_mac->valuestring));
        if(json_device_status)
            cJSON_AddItemToObject(json_sn,"device_status",cJSON_CreateString(json_device_status->valuestring));
        MDF_LOGI("sn: %s",sn);

        p=cJSON_Print(json_sn);
        memset(data, 0, mesh_read_max_length);
        strcpy(data,p);
        if(!cJSON_IsInvalid(json_sn)){
            cJSON_Delete(json_sn);
        }
        if(!cJSON_IsInvalid(json_root)){
            cJSON_Delete(json_root);
        }
        if(p!=NULL)
            free(p);
        if(mode==0){
              xQueueSendFromISR(mesh_Read_Queue, data, NULL);
        }else{
            uart_write_bytes(CONFIG_UART_PORT_NUM, data, strlen(data));//转发给主控芯片
        }
    }

    MDF_LOGW("Node read task is exit");

    MDF_FREE(data);
    vTaskDelete(NULL);
}

static mdf_err_t wifi_init()
{
    mdf_err_t ret          = nvs_flash_init();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        MDF_ERROR_ASSERT(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    MDF_ERROR_ASSERT(ret);

    MDF_ERROR_ASSERT(esp_netif_init());
    MDF_ERROR_ASSERT(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_netif_create_default_wifi_mesh_netifs(&sta_netif, NULL));
    MDF_ERROR_ASSERT(esp_wifi_init(&cfg));
    MDF_ERROR_ASSERT(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
    MDF_ERROR_ASSERT(esp_wifi_set_mode(WIFI_MODE_STA));
    MDF_ERROR_ASSERT(esp_wifi_set_ps(WIFI_PS_NONE));
    MDF_ERROR_ASSERT(esp_mesh_set_6m_rate(false));
    MDF_ERROR_ASSERT(esp_wifi_start());

    return MDF_OK;
}

/**
 * @brief All module events will be sent to this task in esp-mdf
 *
 * @Note:
 *     1. Do not block or lengthy operations in the callback function.
 *     2. Do not consume a lot of memory in the callback function.
 *        The task memory of the callback function is only 4KB.
 */
static mdf_err_t event_loop_cb(mdf_event_loop_t event, void *ctx)
{
    MDF_LOGI("event_loop_cb, event: %d", event);

    switch (event) {
        case MDF_EVENT_MWIFI_STARTED:
            MDF_LOGI("MESH is started");
            break;

        case MDF_EVENT_MWIFI_PARENT_CONNECTED:
            MDF_LOGI("Parent is connected on station interface");
            xTaskCreate(tcp_client_write_task, "tcp_client_write_task", 4 * 1024,
                        NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);
            xTaskCreate(tcp_client_read_task, "tcp_server_read", 4 * 1024,
                        NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);
            if (esp_mesh_is_root()) {
                esp_netif_dhcpc_start(sta_netif);
            }

            break;

        case MDF_EVENT_MWIFI_PARENT_DISCONNECTED:
            MDF_LOGI("Parent is disconnected on station interface");
            break;
        case MDF_EVENT_MWIFI_ROUTING_TABLE_REMOVE:
            MDF_LOGI("total_num: %d", esp_mesh_get_total_node_num());
            break;
        case MDF_EVENT_MWIFI_ROOT_GOT_IP: {
            MDF_LOGI("Root obtains the IP address. It is posted by LwIP stack automatically");
           
            break;
        }
        default:
            break;
    }

    return MDF_OK;
}

void wifi_mesh_init()
{
    mwifi_init_config_t cfg = MWIFI_INIT_CONFIG_DEFAULT();
    char wifi_mac[20];
    mwifi_config_t config   = {
        // .router_ssid     = "baby",
        // .router_password = "1234567890",
        .channel   = 0,
        .mesh_id   = "",
        .mesh_type = 1,
        .channel_switch_disable=1
    };
    if(mode==0){
        strcpy(config.router_ssid,ssid);
        strcpy(config.router_password,pwd);
    }
    config.channel=mesh_channel;
    ESP_LOGD("MESH ID:", "%s", mesh_id);
    memcpy((char *)config.mesh_id,(char *)mesh_id,6);
    config.mesh_type=1;//根节点
    /**
     * @brief Set the log level for serial port printing.
     */
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    /**
     * @brief Initialize wifi mesh.
     */
    MDF_ERROR_ASSERT(mdf_event_loop_init(event_loop_cb));
    MDF_ERROR_ASSERT(wifi_init());
    MDF_ERROR_ASSERT(mwifi_init(&cfg));
    MDF_ERROR_ASSERT(mwifi_set_config(&config));
    MDF_ERROR_ASSERT(mwifi_start());
    /**
     * @brief select/extend a group memebership here
     *      group id can be a custom address
     */
    const uint8_t group_id_list[2][6] = {{0x01, 0x00, 0x5e, 0xae, 0xae, 0xae},
                                         {0x01, 0x00, 0x5e, 0xae, 0xae, 0xaf}};

    MDF_ERROR_ASSERT(esp_mesh_set_group_id((mesh_addr_t *)group_id_list, 
                                sizeof(group_id_list)/sizeof(group_id_list[0])));

    /**
     * @brief Data transfer between wifi mesh devices
     */
    xTaskCreate(node_read_task, "node_read_task", 8 * 1024,
                NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);

    /* Periodic print system information */
    TimerHandle_t timer = xTimerCreate("print_system_info", 10000 / portTICK_RATE_MS,
                                       true, NULL, print_system_info_timercb);
    xTimerStart(timer, 0);

}