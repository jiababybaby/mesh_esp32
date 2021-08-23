
#include "mdf_common.h"
#include "mwifi.h"
#include "driver/uart.h"
#include "socket.h"
#include "mconfig_chain.h"
#include "mesh.h"
static const char *TAG = "ZNWG-MESH";
esp_netif_t *sta_netif;
uint8_t mode;
uint8_t mesh_channel=0;
uint8_t mesh_id[10];
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
/*
开关继电器
in 0 关
   1 开
*/
#define O_PIN 12
static void gpio_deal(uint8_t flag){
    gpio_pad_select_gpio(O_PIN);
    gpio_set_direction(O_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(O_PIN,flag);
}
char send_data[50];
#define mesh_read_max_len 500
static void node_read_task(void *arg)
{
    mdf_err_t ret                    = MDF_OK;
    char *data                       = MDF_MALLOC(mesh_read_max_len);
    size_t size                      = mesh_read_max_len;
    mwifi_data_type_t data_type      = {0x0};
    uint8_t src_addr[MWIFI_ADDR_LEN] = {0x0};

    MDF_LOGI("Node read task is running");

    for (;;) {
        if (!mwifi_is_connected() && !(mwifi_is_started() && esp_mesh_is_root())) {
            vTaskDelay(500 / portTICK_RATE_MS);
            continue;
        }

        size = mesh_read_max_len;
        memset(data, 0, mesh_read_max_len);

        /**
         * @brief Pre-allocated memory to data and size must be specified when passing in a level 1 pointer
         */
        ret = mwifi_read(src_addr, &data_type, data, &size, portMAX_DELAY);
        MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_read", mdf_err_to_name(ret));
        MDF_LOGI("Node receive, addr: " MACSTR ", size: %d, data: %s", MAC2STR(src_addr), size, data);

        /* forwoad to uart */
        // uart_write_bytes(CONFIG_UART_PORT_NUM, data, size);
        // uart_write_bytes(CONFIG_UART_PORT_NUM, "\r\n", 2);

        cJSON *json_root                  = NULL;
        
        json_root = cJSON_Parse(data);
        if(json_root){
            MDF_ERROR_CONTINUE(!json_root, "cJSON_Parse, data format error");
            cJSON *json_device_id  = NULL;
            cJSON *json_data_method  = NULL;
            cJSON *json_data_num  = NULL;
            json_device_id = cJSON_GetObjectItem(json_root, "device_id");
            json_data_method= cJSON_GetObjectItem(json_root, "data_method");
            json_data_num= cJSON_GetObjectItem(json_root, "data_num");
            if(json_device_id&&json_data_method&&json_data_num){
                if(device_kind==jdq){
                    if(!strcmp(json_data_num->valuestring,"0")){
                        gpio_deal(0);
                    }else if(!strcmp(json_data_num->valuestring,"1")){
                        gpio_deal(1);
                    }
                }else if(device_kind==hw){
                    uint8_t device_id=atoi(json_device_id->valuestring);
                    uint8_t num=atoi(json_data_num->valuestring);
                    num=(device_id-1)*8+num;
                    if(!strcmp(json_data_method->valuestring,"o")&&num>=1&&num<=128){
                        uint8_t command[10]={0};
                        command[0]=0xe3;
                        command[1]=num;
                        uart_write_bytes(CONFIG_UART_PORT_NUM, (char *)command, 2);
                    }else if(!strcmp(json_data_method->valuestring,"l")&&num>=1&&num<=128){
                        uint8_t command[10]={0};
                        command[0]=0xe0;
                        command[1]=num;
                        uart_write_bytes(CONFIG_UART_PORT_NUM, (char *)command, 2);
                    }
                }
            }
        }
        if(!cJSON_IsInvalid(json_root)){
            cJSON_Delete(json_root);
        }
    }

    MDF_LOGW("Node read task is exit");

    MDF_FREE(data);
    vTaskDelete(NULL);
}
void write_connect(char *data){
    uint8_t sta_mac[MWIFI_ADDR_LEN] = {0};    
    esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
    char mac_tmp[20]={0};
    sprintf(mac_tmp,MACSTR,sta_mac[0],sta_mac[1],sta_mac[2],sta_mac[3],sta_mac[4],sta_mac[5]);
    MDF_LOGI("%s",mac_tmp);
    cJSON *json_root  = NULL;
    json_root =  cJSON_CreateObject();
    cJSON_AddItemToObject(json_root, "device_mac", cJSON_CreateString((const char *)mac_tmp));
    if(device_kind==jdq)
        cJSON_AddItemToObject(json_root, "device_kind", cJSON_CreateString((const char *)"501"));
    else
        cJSON_AddItemToObject(json_root, "device_kind", cJSON_CreateString((const char *)"502"));
    cJSON_AddItemToObject(json_root, "device_id", cJSON_CreateString((const char *)""));
    cJSON_AddItemToObject(json_root, "device_data", cJSON_CreateString((const char *)"")); 
    cJSON_AddItemToObject(json_root, "device_status", cJSON_CreateString((const char *)"1"));
    char *p=(const char *)cJSON_Print(json_root);
    strcpy(data,p);
    if(!cJSON_IsInvalid(json_root)){
        cJSON_Delete(json_root);
    }
    free(p);
    MDF_LOGI("%s",data);
}
#define BUF_SIZE 500

static void node_write_task(void *arg)
{
    mdf_err_t ret     = MDF_OK;
    // Configure a temporary buffer for the incoming data
    uint8_t *data                     = (uint8_t *) MDF_MALLOC(BUF_SIZE);
    size_t size                       = MWIFI_PAYLOAD_LEN;
    char *jsonstring                  = NULL;
    mwifi_data_type_t data_type       = {0};
    uint8_t sta_mac[MWIFI_ADDR_LEN]   = {0};

    MDF_LOGI("node write handle task is running");

    esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);

    while (1) {
        if (!mwifi_is_connected() && !(mwifi_is_started() && esp_mesh_is_root())) {
            vTaskDelay(500 / portTICK_RATE_MS);
            continue;
        }
        memset(data,0,BUF_SIZE);
        write_connect((char *)data);    
        uint8_t dest_addr[20]=MWIFI_ADDR_ROOT;
        ret = mwifi_write(dest_addr, &data_type, data, strlen((char *)data), true);
        vTaskDelay(10000 / portTICK_RATE_MS);
    }

    MDF_LOGI("node write handle task is exit");

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
           
            if (esp_mesh_is_root()) {
                esp_netif_dhcpc_start(sta_netif);
            }

            break;

        case MDF_EVENT_MWIFI_PARENT_DISCONNECTED:
            MDF_LOGI("Parent is disconnected on station interface");
                MDF_ERROR_ASSERT(mwifi_start());
            break;
        case MDF_EVENT_MWIFI_ROOT_ADDRESS:
           

            break;
        default:
            break;
    }

    return MDF_OK;
}

void wifi_mesh_init()
{

    mwifi_init_config_t cfg = MWIFI_INIT_CONFIG_DEFAULT();
     char wifi_mac[20]="7c9ebd";
    mwifi_config_t config   = {
        // .router_ssid     = "wmdz",
        // .router_password = "wmdz2019",
        .channel   = 13,
        .mesh_id   = "123456",
        .mesh_type = 2,/*0  DEVICE_TYPE_IDLE
                        1  DEVICE_TYPE_ROOT
                        2  DEVICE_TYPE_NODE
                        3  DEVICE_TYPE_LEAF
                        */
        .channel_switch_disable=1
    };
    config.channel=mesh_channel;
    memcpy((char *)config.mesh_id,(char *)mesh_id,6);
    /**
     * @brief Set the log level for serial port printing.
     */
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
    MDF_LOGI("CHANNEL %d, ID %s",config.channel,config.mesh_id);
    /**
     * @brief Initialize wifi mesh.
     */
    MDF_ERROR_ASSERT(mdf_event_loop_init(event_loop_cb));
    MDF_ERROR_ASSERT(wifi_init());
    esp_wifi_set_ps(WIFI_PS_NONE);
    //esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
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
    xTaskCreate(node_read_task, "node_read_task", 4 * 1024,
                NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);
    xTaskCreate(node_write_task, "node_write_task", 4 * 1024,
                NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);

    /* Periodic print system information */
    TimerHandle_t timer = xTimerCreate("print_system_info", 10000 / portTICK_RATE_MS,
                                       true, NULL, print_system_info_timercb);
    xTimerStart(timer, 0);

}