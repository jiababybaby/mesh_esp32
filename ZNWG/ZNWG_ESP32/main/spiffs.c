/* SPIFFS filesystem example.
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "mdf_common.h"
#include "mesh.h"

static const char *TAG = "spiffs";
char canshu[100] = "{\"mode\":1,\"mesh_channel\":1,\"ssid\":\"baby\",\"pwd\":\"1234567890\"}";

/*
data: 待解析的JSON字符串
return: 解析成功的参数的个数（共4个参数）
*/
uint8_t get_info(char *data)
{
    uint8_t ans_val = 0;
    cJSON *json_root = NULL;
    cJSON *json_mode = NULL;
    cJSON *json_MCHL = NULL;
    cJSON *json_ssid = NULL;
    cJSON *json_pwd = NULL;
    json_root = cJSON_Parse((char *)data);
    json_mode = cJSON_GetObjectItem(json_root, "mode");
    if (json_mode)
    {
        ans_val++;
        mode = json_mode->valueint;
        ESP_LOGI(TAG, "get mode %d", mode);
    }
    json_MCHL = cJSON_GetObjectItem(json_root, "mesh_channel");
    if (json_MCHL)
    {
        ans_val++;
        mesh_channel = json_MCHL->valueint;
        ESP_LOGI(TAG, "get mesh_channel %d", mesh_channel);
    }
    json_ssid = cJSON_GetObjectItem(json_root, "ssid");
    if (json_ssid)
    {
        ans_val++;
        strcpy((char *)ssid, (const char *)json_ssid->valuestring);
        ESP_LOGI(TAG, "get ssid %s", ssid);
    }
    json_pwd = cJSON_GetObjectItem(json_root, "pwd");
    if (json_pwd)
    {
        ans_val++;
        strcpy((char *)pwd, (const char *)json_pwd->valuestring);
        ESP_LOGI(TAG, "get pwd %s", pwd);
    }
    if (!cJSON_IsInvalid(json_root))
    {
        cJSON_Delete(json_root);
    }
    return ans_val;
}
void write_info(char *data)
{
    char *p = NULL;
    cJSON *json_root = NULL;
    json_root = cJSON_CreateObject();
    cJSON_AddItemToObject(json_root, "mesh_id", cJSON_CreateString(mesh_id));
    cJSON_AddItemToObject(json_root, "mode", cJSON_CreateNumber(mode));
    cJSON_AddItemToObject(json_root, "mesh_channel", cJSON_CreateNumber(mesh_channel));
    cJSON_AddItemToObject(json_root, "ssid", cJSON_CreateString((const char *)ssid));
    cJSON_AddItemToObject(json_root, "pwd", cJSON_CreateString((const char *)pwd));
    p = cJSON_Print(json_root);
    strcpy(data, p);
    if (!cJSON_IsInvalid(json_root))
    {
        cJSON_Delete(json_root);
    }
    free(p);
}
void write_ans_ok(char *data)
{
    char *p = NULL;
    cJSON *json_root = NULL;
    json_root = cJSON_CreateObject();
    cJSON_AddItemToObject(json_root, "config_ans", cJSON_CreateNumber(1));
    p = cJSON_Print(json_root);
    strcpy(data, p);
    if (!cJSON_IsInvalid(json_root))
    {
        cJSON_Delete(json_root);
    }
    free(p);
}
char data[500];
void save_flash()
{

    write_info(data);
    ESP_LOGI(TAG, "Initializing SPIFFS");

    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true};

    // Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        }
        else if (ret == ESP_ERR_NOT_FOUND)
        {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        }
        else
        {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }

    // Use POSIX and C standard library functions to work with files.
    // First create a file.
    ESP_LOGI(TAG, "Opening file");
    FILE *f;
    f = fopen("/spiffs/hello.txt", "w");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }
    ESP_LOGI(TAG, "Ready to write length %d'", strlen(data));
    uint16_t l_w = fwrite(data, 1, strlen(data), f);
    ESP_LOGI(TAG, "Ready to write length %d'", l_w);
    fclose(f);
    f = fopen("/spiffs/hello.txt", "r");
    char line[500] = {0};
    uint16_t len = 0;
    len = fread(line, 1, sizeof(line), f);
    fclose(f);
    ESP_LOGI(TAG, "Read length %d'", len);
    while (get_info(line) != 4)
    {
        f = fopen("/spiffs/hello.txt", "w");
        if (f == NULL)
        {
            ESP_LOGE(TAG, "Failed to open file for writing");
            return;
        }
        uint16_t w_size = 0;
        w_size = fwrite(data, 1, strlen(data), f);
        if (w_size)
        {
            ESP_LOGI(TAG, "write changed value %s and length %d", data, w_size);
        }
        fclose(f);
        f = fopen("/spiffs/hello.txt", "r");
        len = fread(line, 1, sizeof(line), f);
        fclose(f);
        ESP_LOGI(TAG, "Read from file: '%s'", line);
    }

    ESP_LOGI(TAG, "File written");
    esp_vfs_spiffs_unregister(conf.partition_label);
    ESP_LOGI(TAG, "SPIFFS unmounted");
}
void spi_ffs(void)
{
    ESP_LOGI(TAG, "Initializing SPIFFS");

    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true};

    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        }
        else if (ret == ESP_ERR_NOT_FOUND)
        {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        }
        else
        {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }

    ESP_LOGI(TAG, "Opening file");
    FILE *f = fopen("/spiffs/hello.txt", "r");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for reading");
        f = fopen("/spiffs/hello.txt", "w");
        if (f == NULL)
        {
            ESP_LOGE(TAG, "Failed to open file for writing");
            return;
        }

        fwrite(canshu, 1, strlen(canshu), f);
        fclose(f);
        f = fopen("/spiffs/hello.txt", "r");
    }
    char line[500] = {0};
    uint16_t len = 0;
    len = fread(line, 1, sizeof(line), f);
    fclose(f);
    ESP_LOGI(TAG, "Read length %d'", len);
    while (get_info(line) != 4 || strlen(pwd) < 8)
    {
        f = fopen("/spiffs/hello.txt", "w");
        if (f == NULL)
        {
            ESP_LOGE(TAG, "Failed to open file for writing");
            return;
        }
        uint16_t w_size = 0;
        w_size = fwrite(canshu, 1, strlen(canshu), f);
        if (w_size)
        {
            ESP_LOGI(TAG, "write init value %s and length %d", canshu, w_size);
        }
        fclose(f);
        f = fopen("/spiffs/hello.txt", "r");
        len = fread(line, 1, sizeof(line), f);
        fclose(f);
        ESP_LOGI(TAG, "Read from file: '%s'", line);
        ESP_LOGI(TAG, "Read canshu %d", get_info(line));
    }

    ESP_LOGI(TAG, "File written");
    esp_vfs_spiffs_unregister(conf.partition_label);
    ESP_LOGI(TAG, "SPIFFS unmounted");
}
