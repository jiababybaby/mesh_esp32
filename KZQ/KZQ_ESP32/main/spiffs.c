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

static const char *TAG = "spiffs";
char canshu[100]="{\"mesh_channel\":1,\"mesh_id\":\"123456\"}";
extern uint8_t mesh_channel;
extern char mesh_id[10];
uint8_t get_info(char *data){
    uint8_t ans_val=0;
    cJSON *json_root  = NULL;
    cJSON *json_MCHL  = NULL;
    cJSON *json_MESH_ID  = NULL;
    json_root = cJSON_Parse((char *)data);
    //MDF_ERROR_CONTINUE(!json_root, "cJSON_Parse, data format error, data: %s", data);
    json_MCHL=cJSON_GetObjectItem(json_root, "mesh_channel");
    if(json_MCHL){
        ans_val++;
        mesh_channel=json_MCHL->valueint;
        ESP_LOGI(TAG, "get mesh_channel %d", mesh_channel); 
    }
    json_MESH_ID=cJSON_GetObjectItem(json_root, "mesh_id");
    if(json_MESH_ID){
        ans_val++;
        strcpy((char *)mesh_id,(const char *)json_MESH_ID->valuestring);
        ESP_LOGI(TAG, "get mesh_id %s", mesh_id); 
    }
    if(!cJSON_IsInvalid(json_root)){
        cJSON_Delete(json_root);
    }
    return ans_val;
}
void write_info(char *data){
    cJSON *json_root  = NULL;
    json_root =  cJSON_CreateObject();
    cJSON_AddItemToObject(json_root, "mesh_channel", cJSON_CreateNumber(mesh_channel));

    cJSON_AddItemToObject(json_root, "mesh_id", cJSON_CreateString((const char *)mesh_id));

    strcpy(data,(const char *)cJSON_Print(json_root));
    if(!cJSON_IsInvalid(json_root)){
        cJSON_Delete(json_root);
    }
}
void write_ans_ok(char *data){
    cJSON *json_root  = NULL;
    json_root =  cJSON_CreateObject();
    cJSON_AddItemToObject(json_root, "config_ans", cJSON_CreateNumber(1));
    strcpy(data,(const char *)cJSON_Print(json_root));
    if(!cJSON_IsInvalid(json_root)){
        cJSON_Delete(json_root);
    }
}
void save_flash(){
    char data[100];
    write_info(data);
    ESP_LOGI(TAG, "Initializing SPIFFS");
    
    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = NULL,
      .max_files = 5,
      .format_if_mount_failed = true
    };
    
    // Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }
    
    size_t total = 0, used = 0;
    ret = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }

    // Use POSIX and C standard library functions to work with files.
    // First create a file.
    ESP_LOGI(TAG, "Opening file");
    FILE* f ;
    f = fopen("/spiffs/hello.txt", "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }
    
    fwrite(data,1,strlen(data),f); 
    fclose(f);
    f = fopen("/spiffs/hello.txt", "r");
    char line[64]={0};
    uint16_t len=0;
    len=fread(line, 1, sizeof(line), f);
    fclose(f);
    ESP_LOGI(TAG, "Read length %d'", len);   
    while(get_info(line)!=2){
        f = fopen("/spiffs/hello.txt", "w");
        if (f == NULL) {
            ESP_LOGE(TAG, "Failed to open file for writing");
            return;
        }
        uint16_t w_size=0;
        w_size=fwrite(data,1,strlen(data),f);
        if(w_size){
            ESP_LOGI(TAG, "write changed value %s and length %d", data,w_size);   
        } 
        fclose(f);
        f = fopen("/spiffs/hello.txt", "r");
        len=fread(line, 1, sizeof(line), f);
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
      .format_if_mount_failed = true
    };
    
    // Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }
    
    size_t total = 0, used = 0;
    ret = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }

    // Use POSIX and C standard library functions to work with files.
    // First create a file.
    ESP_LOGI(TAG, "Opening file");
    FILE* f = fopen("/spiffs/hello.txt", "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        f = fopen("/spiffs/hello.txt", "w");
        if (f == NULL) {
            ESP_LOGE(TAG, "Failed to open file for writing");
            return;
        }
       
        fwrite(canshu,1,strlen(canshu),f); 
        fclose(f);
        f = fopen("/spiffs/hello.txt", "r");
    }   
    char line[64]={0};
    uint16_t len=0;
    len=fread(line, 1, sizeof(line), f);
    fclose(f);
    ESP_LOGI(TAG, "Read length %d'", len);   
    while(get_info(line)!=2){
        f = fopen("/spiffs/hello.txt", "w");
        if (f == NULL) {
            ESP_LOGE(TAG, "Failed to open file for writing");
            return;
        }
        uint16_t w_size=0;
        w_size=fwrite(canshu,1,strlen(canshu),f);
        if(w_size){
            ESP_LOGI(TAG, "write init value %s and length %d", canshu,w_size);   
        } 
        fclose(f);
        f = fopen("/spiffs/hello.txt", "r");
        len=fread(line, 1, sizeof(line), f);
        fclose(f);
        ESP_LOGI(TAG, "Read from file: '%s'", line);   
    }

    ESP_LOGI(TAG, "File written");

    // Check if destination file exists before renaming
    // struct stat st;
    // if (stat("/spiffs/foo.txt", &st) == 0) {
    //     // Delete it if it exists
    //     unlink("/spiffs/foo.txt");
    // }

    // // Rename original file
    // ESP_LOGI(TAG, "Renaming file");
    // if (rename("/spiffs/hello.txt", "/spiffs/foo.txt") != 0) {
    //     ESP_LOGE(TAG, "Rename failed");
    //     return;
    // }

    // Open renamed file for reading
    // ESP_LOGI(TAG, "Reading file");
    // f = fopen("/spiffs/foo.txt", "r");
    // if (f == NULL) {
    //     ESP_LOGE(TAG, "Failed to open file for reading");
    //     return;
    // }
    // char line[64];
    // fgets(line, sizeof(line), f);
    // fclose(f);
    // strip newline
    // char* pos = strchr(line, '\n');
    // if (pos) {
    //     *pos = '\0';
    // }
    // ESP_LOGI(TAG, "Read from file: '%s'", line);

    // All done, unmount partition and disable SPIFFS
    esp_vfs_spiffs_unregister(conf.partition_label);
    ESP_LOGI(TAG, "SPIFFS unmounted");
}
