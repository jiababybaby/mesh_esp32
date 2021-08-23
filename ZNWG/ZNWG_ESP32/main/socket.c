
#include "mdf_common.h"
#include "mwifi.h"
#include "mesh.h"
static int g_sockfd    = -1;
static const char *TAG = "ZNWG-SOCKET";
#define TCP_WRITE_MAX_LEN 150
#define TCP_READ_MAX_LEN 150
/**
 * @brief 创建TCP连接服务，返回TCP服务句柄
 */
static int socket_tcp_client_create(const char *ip, uint16_t port)
{
    MDF_PARAM_CHECK(ip);

    MDF_LOGI("Create a tcp client, ip: %s, port: %d", ip, port);

    mdf_err_t ret = ESP_OK;
    int sockfd    = -1;
    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(port),
        .sin_addr.s_addr = inet_addr(ip),
    };

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    MDF_ERROR_GOTO(sockfd < 0, ERR_EXIT, "socket create, sockfd: %d", sockfd);

    ret = connect(sockfd, (struct sockaddr *)&server_addr, sizeof(struct sockaddr_in));
    MDF_ERROR_GOTO(ret < 0, ERR_EXIT, "socket connect, ret: %d, ip: %s, port: %d",
                   ret, ip, port);
    return sockfd;

ERR_EXIT:

    if (sockfd != -1) {
        close(sockfd);
    }

    return -1;
}
/*
TCP读任务
*/
uint8_t tcp_init_flag=0;
void tcp_client_read_task(void *arg)
{
    mdf_err_t ret                     = MDF_OK;
    char *data                        = MDF_MALLOC(TCP_READ_MAX_LEN);
    size_t size                       = TCP_READ_MAX_LEN;
    uint8_t dest_addr[MWIFI_ADDR_LEN] = {0x0};
    mwifi_data_type_t data_type       = {0x0};
    cJSON *json_root                  = NULL;
    cJSON *json_addr                  = NULL;
    cJSON *json_group                 = NULL;
    cJSON *json_data                  = NULL;
    cJSON *json_dest_addr             = NULL;

    MDF_LOGI("TCP client read task is running");

    while (mwifi_is_connected()) {
        if (g_sockfd == -1) {
            g_sockfd = socket_tcp_client_create("119.29.196.53", 1234);

            if (g_sockfd == -1) {
                vTaskDelay(500 / portTICK_RATE_MS);
                continue;
            }
        }
        // ret = read(g_sockfd, data, size);
        memset(data, 0, TCP_READ_MAX_LEN);
        ret = read(g_sockfd, data, size);
        MDF_LOGI("TCP read, %d, size: %d, data: %s", g_sockfd, size, data);

        if (ret <= 0) {
            MDF_LOGW("<%s> TCP read", strerror(errno));
            close(g_sockfd);
            g_sockfd = -1;
            tcp_init_flag=0;
            continue;
        }

        json_root = cJSON_Parse(data);
        MDF_ERROR_CONTINUE(!json_root, "cJSON_Parse, data format error");

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

    MDF_LOGI("TCP client read task is exit");

    close(g_sockfd);
    g_sockfd = -1;
    MDF_FREE(data);
    vTaskDelete(NULL);
}
/*
TCP写任务
*/
extern QueueHandle_t mesh_Read_Queue;
void tcp_client_write_task(void *arg)
{
    mdf_err_t ret = MDF_OK;
    char *data    = MDF_CALLOC(1, TCP_WRITE_MAX_LEN);
    size_t size   = TCP_WRITE_MAX_LEN;
    uint8_t src_addr[MWIFI_ADDR_LEN] = {0x0};
    mwifi_data_type_t data_type      = {0x0};

    MDF_LOGI("TCP client write task is running");

    while (mwifi_is_connected()) {
        if (g_sockfd == -1) {
            vTaskDelay(500 / portTICK_RATE_MS);
            continue;
        }
        if(tcp_init_flag==0){
            tcp_init_flag=1;
            memset(data, 0, TCP_READ_MAX_LEN);
            char *p=NULL;
            cJSON *json_root  = NULL;
            json_root =  cJSON_CreateObject();
            cJSON_AddItemToObject(json_root, "sn", cJSON_CreateString(mesh_id));
            cJSON_AddItemToObject(json_root, "sn_status", cJSON_CreateNumber(1));
            p=cJSON_Print(json_root);
            strcpy(data,p);
            if(!cJSON_IsInvalid(json_root)){
                cJSON_Delete(json_root);
            }
            write(g_sockfd,data,strlen(data));
            free(p);

        }
        if(xQueueReceive(mesh_Read_Queue, data, portMAX_DELAY)) {
            size=strlen(data);
            MDF_LOGD("TCP write, size: %d, data: %s", size, data);
            ret = write(g_sockfd, data, size);
            MDF_ERROR_CONTINUE(ret <= 0, "<%s> TCP write", strerror(errno));
        }
    }

    MDF_LOGI("TCP client write task is exit");

    close(g_sockfd);
    g_sockfd = -1;
    MDF_FREE(data);
    vTaskDelete(NULL);
}