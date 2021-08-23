
#include "mdf_common.h"
#include "mwifi.h"
#include "driver/uart.h"
#include "socket.h"
#include "iic.h"
#include "mesh.h"
static const char *TAG = "ZNWG-MESH";
esp_netif_t *sta_netif;
uint8_t mode;
uint8_t mesh_channel=0;
char mesh_id[10];
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
static void node_read_task(void *arg)
{
    mdf_err_t ret                    = MDF_OK;
    char *data                       = MDF_MALLOC(MWIFI_PAYLOAD_LEN);
    size_t size                      = MWIFI_PAYLOAD_LEN;
    mwifi_data_type_t data_type      = {0x0};
    uint8_t src_addr[MWIFI_ADDR_LEN] = {0x0};

    MDF_LOGI("Node read task is running");

    for (;;) {
        if (!mwifi_is_connected() && !(mwifi_is_started() && esp_mesh_is_root())) {
            vTaskDelay(500 / portTICK_RATE_MS);
            continue;
        }

        size = MWIFI_PAYLOAD_LEN;
        memset(data, 0, MWIFI_PAYLOAD_LEN);

        /**
         * @brief Pre-allocated memory to data and size must be specified when passing in a level 1 pointer
         */
        ret = mwifi_read(src_addr, &data_type, data, &size, portMAX_DELAY);
        MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_read", mdf_err_to_name(ret));
        MDF_LOGI("Node receive, addr: " MACSTR ", size: %d, data: %s", MAC2STR(src_addr), size, data);

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
    if(device_kind==dht11)
        cJSON_AddItemToObject(json_root, "device_kind", cJSON_CreateString((const char *)"1"));
    else
        cJSON_AddItemToObject(json_root, "device_kind", cJSON_CreateString((const char *)"502"));
    cJSON_AddItemToObject(json_root, "device_id", cJSON_CreateString((const char *)"0"));
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
void write_data(char *data, uint8_t device_id, char* device_data){
    uint8_t sta_mac[MWIFI_ADDR_LEN] = {0};    
    esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
    char mac_tmp[20]={0};
    sprintf(mac_tmp,MACSTR,sta_mac[0],sta_mac[1],sta_mac[2],sta_mac[3],sta_mac[4],sta_mac[5]);
    MDF_LOGI("%s",mac_tmp);
    
    char device_kind_str[20]={0};
    switch(device_id){
        case 1:
            strcat(device_kind_str,"1");
            break;
        case 2:
            strcat(device_kind_str,"1");
            break;
        case 3:
            strcat(device_kind_str,"2");
            break;
        case 4:
            strcat(device_kind_str,"3");
            break;
        case 5:
        case 6:
        case 7:
            strcat(device_kind_str,"4");
            break;
        case 8:
        case 9:
            strcat(device_kind_str,"5");
            break;
        case 10:
            strcat(device_kind_str,"6");
            break;
        case 11:
            strcat(device_kind_str,"7");
            break;
        case 12:
        case 13:
        case 14:
        case 15:
        case 16:
        case 17:
        case 18:
        case 19:
            strcat(device_kind_str,"8");
            break;
        default:
            break;

    }
    char device_id_temp[20]={0};
    sprintf(device_id_temp,"%d",device_id);


    cJSON *json_root  = NULL;
    json_root =  cJSON_CreateObject();
    cJSON_AddItemToObject(json_root, "device_mac", cJSON_CreateString((const char *)mac_tmp));
    cJSON_AddItemToObject(json_root, "device_kind", cJSON_CreateString((const char *)device_kind_str));
    cJSON_AddItemToObject(json_root, "device_id", cJSON_CreateString((const char *)device_id_temp));
    cJSON_AddItemToObject(json_root, "device_data", cJSON_CreateString((const char *)device_data)); 
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
extern char temp_val[20],hum_val[20],zp13_val[20],zp15_val[20];
extern char pm_1_0_val[20],pm_2_5_val[20],pm_10_val[20];
extern char tvoc_val[20],eco2_val[20];
extern char lux_val[20];
extern char w_val[20];
extern char p_v_val[20],p_i_val[20],
p_p_val[20],p_q_val[20],p_s_val[20],
p_pf_val[20],p_f_val[20],p_ep_val[20];
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
    char* get_val[30];
    get_val[0]=temp_val;
    get_val[1]=hum_val;
    get_val[2]=zp13_val;
    get_val[3]=zp15_val;
    get_val[4]=pm_1_0_val;
    get_val[5]=pm_2_5_val;
    get_val[6]=pm_10_val;
    get_val[7]=tvoc_val;
    get_val[8]=eco2_val;
    get_val[9]=lux_val;
    get_val[10]=w_val;
    get_val[11]=p_ep_val;
    get_val[12]=p_v_val;
    get_val[13]=p_i_val;
    get_val[14]=p_p_val;
    get_val[15]=p_q_val;
    get_val[16]=p_s_val;
    get_val[17]=p_pf_val;
    get_val[18]=p_f_val;
    get_val[19]=temp_val;
    get_val[20]=temp_val;
    while (1) {
        if (!mwifi_is_connected() && !(mwifi_is_started() && esp_mesh_is_root())) {
            vTaskDelay(500 / portTICK_RATE_MS);
            continue;
        }
        
        for(int i=0;i<19;i++){
            memset(data,0,BUF_SIZE);
            if(get_val[i])
                MDF_LOGI("%s",get_val[i]);
            write_data((char *)data,i+1,get_val[i]);   
            uint8_t dest_addr[20]=MWIFI_ADDR_ROOT;
            ret = mwifi_write(dest_addr, &data_type, data, strlen((char *)data), true);
            vTaskDelay(1000 / portTICK_RATE_MS);
        }
        write_connect((char *)data); 
  

      
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

            if (esp_mesh_is_root()) {
                esp_netif_dhcpc_start(sta_netif);
            }

            break;

        case MDF_EVENT_MWIFI_PARENT_DISCONNECTED:
            MDF_LOGI("Parent is disconnected on station interface");
                MDF_ERROR_ASSERT(mwifi_start());
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
        // .router_ssid     = "baby",
        // .router_password = "1234567890",
        .channel   = 0,
        .mesh_id   = "",
        .mesh_type = 2,/*0  DEVICE_TYPE_IDLE
                        1  DEVICE_TYPE_ROOT
                        2  DEVICE_TYPE_NODE
                        3  DEVICE_TYPE_LEAF
                        */
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