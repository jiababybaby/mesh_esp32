// Copyright 2016-2019 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "string.h"
#include "esp_log.h"
#include "modbus_params.h"  // for modbus parameters structures
#include "mbcontroller.h"
#include "sdkconfig.h"
#include "mdf_common.h"
#include "mwifi.h"
#include "driver/uart.h"
#include "socket.h"
#include "iic.h"
#include "mesh.h"
#define MB_PORT_NUM     (2)   // Number of UART port used for Modbus connection
#define MB_DEV_SPEED    (9600)  // The communication speed of the UART

// Note: Some pins on target chip cannot be assigned for UART communication.
// See UART documentation for selected board and target to configure pins using Kconfig.

// The number of parameters that intended to be used in the particular control process
#define MASTER_MAX_CIDS num_device_parameters

// Number of reading of parameters from slave
#define MASTER_MAX_RETRY 30

// Timeout to update cid over Modbus
#define UPDATE_CIDS_TIMEOUT_MS          (2000)
#define UPDATE_CIDS_TIMEOUT_TICS        (UPDATE_CIDS_TIMEOUT_MS / portTICK_RATE_MS)

// Timeout between polls
#define POLL_TIMEOUT_MS                 (1000)
#define POLL_TIMEOUT_TICS               (POLL_TIMEOUT_MS / portTICK_RATE_MS)

#define MASTER_TAG "JCQ_MASTER"

#define MASTER_CHECK(a, ret_val, str, ...) \
    if (!(a)) { \
        ESP_LOGE(MASTER_TAG, "%s(%u): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
        return (ret_val); \
    }

// The macro to get offset for parameter in the appropriate structure
#define HOLD_OFFSET(field) ((uint16_t)(offsetof(holding_reg_params_t, field) + 1))
#define INPUT_OFFSET(field) ((uint16_t)(offsetof(input_reg_params_t, field) + 1))
#define COIL_OFFSET(field) ((uint16_t)(offsetof(coil_reg_params_t, field) + 1))
// Discrete offset macro
#define DISCR_OFFSET(field) ((uint16_t)(offsetof(discrete_reg_params_t, field) + 1))

#define STR(fieldname) ((const char*)( fieldname ))
// Options can be used as bit masks or parameter limits
#define OPTS(min_val, max_val, step_val) { .opt1 = min_val, .opt2 = max_val, .opt3 = step_val }

// Enumeration of modbus device addresses accessed by master device
enum {
    MB_DEVICE_WATER = 1,// Only one slave device used for the test (add other slave addresses here)
    MB_DEVICE_POWER
};

// Enumeration of all supported CIDs for device (used in parameter definition table)
enum {
    ADDR_WATER_HOLD_DATA=0X00,//用水量，使用两个寄存器，四个字节
    ADDR_WATER_HOLD_Addr_BAU=0X15,
    ADDR_POWER_HOLD_CLEAN=0x0002,
    ADDR_POWER_HOLD_PROTO=0x0005,
    ADDR_POWER_HOLD_ADDR=0x0006,
    ADDR_POWER_HOLD_BAU=0x000C,
    ADDR_POWER_HOLD_V=0x2000,//电压，使用两个寄存器，四个字节
    ADDR_POWER_HOLD_I=0x2006,//电流，使用两个寄存器，四个字节
    ADDR_POWER_HOLD_P=0x2002,//瞬时总有功功率，使用两个寄存器，四个字节
    ADDR_POWER_HOLD_Q=0x2004,//瞬时总无功功率，使用两个寄存器，四个字节
    ADDR_POWER_HOLD_S=0x2008,//瞬时总视在功率，使用两个寄存器，四个字节
    ADDR_POWER_HOLD_PF=0x200A,//总功功率因数，使用两个寄存器，四个字节
    ADDR_POWER_HOLD_F=0x200E,//频率，使用两个寄存器，四个字节
    ADDR_POWER_HOLD_EP=0x4000,//有功总电能，使用两个寄存器，四个字节
};
enum {
    CID_WATER_HOLD_DATA,//用水量，使用两个寄存器，四个字节
    CID_WATER_HOLD_ADDR_BAU,
    CID_POWER_HOLD_CLEAN,
    CID_POWER_HOLD_PROTO,
    CID_POWER_HOLD_ADDR,
    CID_POWER_HOLD_BAU,
    CID_POWER_HOLD_V,//电压，使用两个寄存器，四个字节
    CID_POWER_HOLD_I,//电流，使用两个寄存器，四个字节
    CID_POWER_HOLD_P,//瞬时总有功功率，使用两个寄存器，四个字节
    CID_POWER_HOLD_Q,//瞬时总无功功率，使用两个寄存器，四个字节
    CID_POWER_HOLD_S,//瞬时总视在功率，使用两个寄存器，四个字节
    CID_POWER_HOLD_PF,//总功功率因数，使用两个寄存器，四个字节
    CID_POWER_HOLD_F,//频率，使用两个寄存器，四个字节
    CID_POWER_HOLD_EP,//有功总电能，使用两个寄存器，四个字节
    CID_HOLD_TEST_REG
};

// Example Data (Object) Dictionary for Modbus parameters:
// The CID field in the table must be unique.
// Modbus Slave Addr field defines slave address of the device with correspond parameter.
// Modbus Reg Type - Type of Modbus register area (Holding register, Input Register and such).
// Reg Start field defines the start Modbus register number and Reg Size defines the number of registers for the characteristic accordingly.
// The Instance Offset defines offset in the appropriate parameter structure that will be used as instance to save parameter value.
// Data Type, Data Size specify type of the characteristic and its data size.
// Parameter Options field specifies the options that can be used to process parameter value (limits or masks).
// Access Mode - can be used to implement custom options for processing of characteristic (Read/Write restrictions, factory mode values and etc).
const mb_parameter_descriptor_t device_parameters[] = {
    // { CID, Param Name, Units, Modbus Slave Addr, Modbus Reg Type, Reg Start, Reg Size, Instance Offset, Data Type, Data Size, Parameter Options, Access Mode}
    { CID_WATER_HOLD_DATA, STR("WATER_DATA"), STR("m3"), MB_DEVICE_WATER, MB_PARAM_HOLDING, ADDR_WATER_HOLD_DATA, 2,
            HOLD_OFFSET(WATER_DATA), PARAM_TYPE_ASCII, 4, OPTS( 0, 100000, 1 ), PAR_PERMS_READ_WRITE_TRIGGER },
    { CID_WATER_HOLD_ADDR_BAU, STR("WATER_BAU"), STR(""), MB_DEVICE_POWER, MB_PARAM_HOLDING, ADDR_WATER_HOLD_Addr_BAU, 2,
            HOLD_OFFSET(WATER_BAU), PARAM_TYPE_ASCII, 4, OPTS( 0, 100, 1 ), PAR_PERMS_READ_WRITE_TRIGGER },
    { CID_POWER_HOLD_CLEAN, STR("POWER_C"), STR("C"), MB_DEVICE_POWER, MB_PARAM_HOLDING, ADDR_POWER_HOLD_CLEAN, 2,
            HOLD_OFFSET(POWER_C), PARAM_TYPE_ASCII, 4, OPTS( -40, 100, 1 ), PAR_PERMS_READ_WRITE_TRIGGER },
    { CID_POWER_HOLD_PROTO, STR("POWER_PROTO"), STR("%rH"), MB_DEVICE_POWER, MB_PARAM_HOLDING, ADDR_POWER_HOLD_PROTO, 2,
            HOLD_OFFSET(POWER_PROTO), PARAM_TYPE_ASCII, 4, OPTS( 0, 100, 1 ), PAR_PERMS_READ_WRITE_TRIGGER },
    { CID_POWER_HOLD_ADDR, STR("POWER_ADDR"), STR("C"), MB_DEVICE_POWER, MB_PARAM_HOLDING, ADDR_POWER_HOLD_ADDR, 2,
            HOLD_OFFSET(POWER_ADDR), PARAM_TYPE_ASCII, 4, OPTS( -40, 100, 1 ), PAR_PERMS_READ_WRITE_TRIGGER },
    { CID_POWER_HOLD_BAU, STR("POWER_BAU"), STR("%rH"), MB_DEVICE_POWER, MB_PARAM_HOLDING, ADDR_POWER_HOLD_BAU, 2,
            HOLD_OFFSET(POWER_BAU), PARAM_TYPE_ASCII, 4, OPTS( 0, 100, 1 ), PAR_PERMS_READ_WRITE_TRIGGER },
    { CID_POWER_HOLD_V, STR("POWER_V"), STR("__"), MB_DEVICE_POWER, MB_PARAM_HOLDING, ADDR_POWER_HOLD_V, 2,
            HOLD_OFFSET(POWER_V), PARAM_TYPE_ASCII, 4, OPTS( 0, 100, 1 ), PAR_PERMS_READ_WRITE_TRIGGER },
    { CID_POWER_HOLD_I, STR("POWER_I"), STR("on/off"), MB_DEVICE_POWER, MB_PARAM_HOLDING, ADDR_POWER_HOLD_I, 2,
            HOLD_OFFSET(POWER_I), PARAM_TYPE_ASCII, 4, OPTS( 0, 100000, 1 ), PAR_PERMS_READ_WRITE_TRIGGER },
    { CID_POWER_HOLD_P, STR("POWER_P"), STR("on/off"), MB_DEVICE_POWER, MB_PARAM_HOLDING, ADDR_POWER_HOLD_P, 2,
            HOLD_OFFSET(POWER_P), PARAM_TYPE_ASCII, 4, OPTS( 0, 100000, 1 ), PAR_PERMS_READ_WRITE_TRIGGER },
    { CID_POWER_HOLD_Q, STR("POWER_Q"), STR("on/off"), MB_DEVICE_POWER, MB_PARAM_HOLDING, ADDR_POWER_HOLD_Q, 2,
            HOLD_OFFSET(POWER_Q), PARAM_TYPE_ASCII, 4, OPTS( 0, 100000, 1 ), PAR_PERMS_READ_WRITE_TRIGGER },
    { CID_POWER_HOLD_S, STR("POWER_S"), STR("on/off"), MB_DEVICE_POWER, MB_PARAM_HOLDING, ADDR_POWER_HOLD_S, 2,
            HOLD_OFFSET(POWER_S), PARAM_TYPE_ASCII, 4, OPTS( 0, 100000, 1 ), PAR_PERMS_READ_WRITE_TRIGGER },
    { CID_POWER_HOLD_PF, STR("POWER_PF"), STR("on/off"), MB_DEVICE_POWER, MB_PARAM_HOLDING, ADDR_POWER_HOLD_PF, 2,
            HOLD_OFFSET(POWER_PF), PARAM_TYPE_ASCII, 4, OPTS( 0, 100000, 1 ), PAR_PERMS_READ_WRITE_TRIGGER },
    { CID_POWER_HOLD_F, STR("POWER_F"), STR("on/off"), MB_DEVICE_POWER, MB_PARAM_HOLDING, ADDR_POWER_HOLD_F, 2,
            HOLD_OFFSET(POWER_F), PARAM_TYPE_ASCII, 4, OPTS( 0, 100000, 1 ), PAR_PERMS_READ_WRITE_TRIGGER },
    { CID_POWER_HOLD_EP, STR("POWER_EP"), STR("on/off"), MB_DEVICE_POWER, MB_PARAM_HOLDING, ADDR_POWER_HOLD_EP, 2,
            HOLD_OFFSET(POWER_EP), PARAM_TYPE_ASCII, 4, OPTS( 0, 100000, 1 ), PAR_PERMS_READ_WRITE_TRIGGER }

};

// Calculate number of parameters in the table
const uint16_t num_device_parameters = (sizeof(device_parameters)/sizeof(device_parameters[0]));

// The function to get pointer to parameter storage (instance) according to parameter description table
static void* master_get_param_data(const mb_parameter_descriptor_t* param_descriptor)
{
    assert(param_descriptor != NULL);
    void* instance_ptr = NULL;
    if (param_descriptor->param_offset != 0) {
       switch(param_descriptor->mb_param_type)
       {
           case MB_PARAM_HOLDING:
               instance_ptr = ((void*)&holding_reg_params + param_descriptor->param_offset - 1);
               break;
           case MB_PARAM_INPUT:
               instance_ptr = ((void*)&input_reg_params + param_descriptor->param_offset - 1);
               break;
           case MB_PARAM_COIL:
               instance_ptr = ((void*)&coil_reg_params + param_descriptor->param_offset - 1);
               break;
           case MB_PARAM_DISCRETE:
               instance_ptr = ((void*)&discrete_reg_params + param_descriptor->param_offset - 1);
               break;
           default:
               instance_ptr = NULL;
               break;
       }
    } else {
        ESP_LOGE(MASTER_TAG, "Wrong parameter offset for CID #%d", param_descriptor->cid);
        assert(instance_ptr != NULL);
    }
    return instance_ptr;
}
union FLOAT{
    float f;
    unsigned char h[4];
} ;
// User operation function to read slave values and check alarm
char w_val[20]={0};
char p_v_val[20]={0},p_i_val[20]={0},
p_p_val[20]={0},p_q_val[20]={0},p_s_val[20]={0},
p_pf_val[20]={0},p_f_val[20]={0},p_ep_val[20]={0};
static void master_operation_func(void *arg)
{
    esp_err_t err = ESP_OK;
    float value = 0;
    int val=0;
    bool alarm_state = false;
    const mb_parameter_descriptor_t* param_descriptor = NULL;
    
    ESP_LOGI(MASTER_TAG, "Start modbus test...");
    
    for(; ; ) {
        // Read all found characteristics from slave(s)
        for (uint16_t cid = 0; (err != ESP_ERR_NOT_FOUND) && cid < MASTER_MAX_CIDS; cid++) 
        {
            // Get data from parameters description table
            // and use this information to fill the characteristics description table
            // and having all required fields in just one table
            err = mbc_master_get_cid_info(cid, &param_descriptor);
            if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
                void* temp_data_ptr = master_get_param_data(param_descriptor);
                assert(temp_data_ptr);
                uint8_t type = 0;
                err = mbc_master_get_parameter(cid, (char*)param_descriptor->param_key,
                                                        (uint8_t*)&val, &type);
                if (err == ESP_OK) {
                    if(!strcmp(param_descriptor->param_key,"WATER_DATA")){
                        float water_tmp=(float)((val&0xffff0000)>>16|(val&0x0000ffff)<<16)/100;
                        ESP_LOGI(MASTER_TAG, "water %f",water_tmp);
                        memset(w_val,0,sizeof(w_val));
                        sprintf(w_val,"%f",water_tmp);
                    }
                    if(!strcmp(param_descriptor->param_key,"POWER_V")){
                        union FLOAT f;
                        f.h[3]=(val&0x0000ff00)>>8;
                        f.h[2]=val&0x000000ff;
                        f.h[1]=(val&0xff000000)>>24;
                        f.h[0]=(val&0x00ff0000)>>16;
                        float p_v=f.f;
                        ESP_LOGI(MASTER_TAG, "power_v %x %f",val,p_v);
                        memset(p_v_val,0,sizeof(p_v_val));
                        sprintf(p_v_val,"%f",p_v);                    
                    }else
                    if(!strcmp(param_descriptor->param_key,"POWER_I")){
                        union FLOAT f;
                        f.h[3]=(val&0x0000ff00)>>8;
                        f.h[2]=val&0x000000ff;
                        f.h[1]=(val&0xff000000)>>24;
                        f.h[0]=(val&0x00ff0000)>>16;
                        float p_i=f.f;
                        ESP_LOGI(MASTER_TAG, "POWER_I %x %f",val,p_i);
                        memset(p_i_val,0,sizeof(p_i_val));
                        sprintf(p_i_val,"%f",p_i);    
                    }else
                    if(!strcmp(param_descriptor->param_key,"POWER_P")){
                        union FLOAT f;
                        f.h[3]=(val&0x0000ff00)>>8;
                        f.h[2]=val&0x000000ff;
                        f.h[1]=(val&0xff000000)>>24;
                        f.h[0]=(val&0x00ff0000)>>16;
                        float p_p=f.f;
                        ESP_LOGI(MASTER_TAG, "POWER_P %x %f",val,p_p);
                        memset(p_p_val,0,sizeof(p_p_val));
                        sprintf(p_p_val,"%f",p_p);                        
                    }else
                    if(!strcmp(param_descriptor->param_key,"POWER_Q")){
                        union FLOAT f;
                        f.h[3]=(val&0x0000ff00)>>8;
                        f.h[2]=val&0x000000ff;
                        f.h[1]=(val&0xff000000)>>24;
                        f.h[0]=(val&0x00ff0000)>>16;
                        float p_q=f.f;
                        ESP_LOGI(MASTER_TAG, "POWER_Q %x %f",val,p_q);
                        memset(p_q_val,0,sizeof(p_q_val));
                        sprintf(p_q_val,"%f",p_q);    
                    }else
                    if(!strcmp(param_descriptor->param_key,"POWER_S")){
                        union FLOAT f;
                        f.h[3]=(val&0x0000ff00)>>8;
                        f.h[2]=val&0x000000ff;
                        f.h[1]=(val&0xff000000)>>24;
                        f.h[0]=(val&0x00ff0000)>>16;
                        float p_s=f.f;
                        ESP_LOGI(MASTER_TAG, "POWER_S %x %f",val,p_s);
                        memset(p_s_val,0,sizeof(p_s_val));
                        sprintf(p_s_val,"%f",p_s);    
                    }else
                    if(!strcmp(param_descriptor->param_key,"POWER_PF")){
                        union FLOAT f;
                        f.h[3]=(val&0x0000ff00)>>8;
                        f.h[2]=val&0x000000ff;
                        f.h[1]=(val&0xff000000)>>24;
                        f.h[0]=(val&0x00ff0000)>>16;
                        float p_pf=f.f;
                        ESP_LOGI(MASTER_TAG, "POWER_PF %x %f",val,p_pf);
                        memset(p_pf_val,0,sizeof(p_pf_val));
                        sprintf(p_pf_val,"%f",p_pf);    
                    }else
                    if(!strcmp(param_descriptor->param_key,"POWER_F")){
                        union FLOAT f;
                        f.h[3]=(val&0x0000ff00)>>8;
                        f.h[2]=val&0x000000ff;
                        f.h[1]=(val&0xff000000)>>24;
                        f.h[0]=(val&0x00ff0000)>>16;
                        float p_f=f.f;
                        ESP_LOGI(MASTER_TAG, "POWER_F %x %f",val,p_f);
                        memset(p_f_val,0,sizeof(p_f_val));
                        sprintf(p_f_val,"%f",p_f);    
                    }else                    
                    if(!strcmp(param_descriptor->param_key,"POWER_EP")){
                        union FLOAT f;
                        f.h[3]=(val&0x0000ff00)>>8;
                        f.h[2]=val&0x000000ff;
                        f.h[1]=(val&0xff000000)>>24;
                        f.h[0]=(val&0x00ff0000)>>16;
                        float p_ep=f.f;
                        ESP_LOGI(MASTER_TAG, "POWER_EP %x %f",val,p_ep);
                        memset(p_ep_val,0,sizeof(p_ep_val));
                        sprintf(p_ep_val,"%f",p_ep);    
                    }     
                } else {
                    ESP_LOGE(MASTER_TAG, "Characteristic #%d (%s) read fail, err = 0x%x (%s).",
                                        param_descriptor->cid,
                                        (char*)param_descriptor->param_key,
                                        (int)err,
                                        (char*)esp_err_to_name(err));
                }
                
                vTaskDelay(POLL_TIMEOUT_TICS); // timeout between polls
            }
        }
        vTaskDelay(UPDATE_CIDS_TIMEOUT_TICS); // 
    }
    
    if (alarm_state) {   
        ESP_LOGI(MASTER_TAG, "Alarm triggered by cid #%d.",
                                        param_descriptor->cid);
    } else {
        ESP_LOGE(MASTER_TAG, "Alarm is not triggered after %d retries.",
                                        MASTER_MAX_RETRY);
    }
    ESP_LOGI(MASTER_TAG, "Destroy master...");
    ESP_ERROR_CHECK(mbc_master_destroy());
}
#define mb_port 2
// Modbus master initialization
static esp_err_t master_init(void)
{
    if(uart_is_driver_installed(mb_port)){
        uart_driver_delete(mb_port);
    }
    // Initialize and start Modbus controller
    mb_communication_info_t comm = {
            .port = mb_port,
            .mode = MB_MODE_RTU,
            .baudrate = 9600,
            .parity = MB_PARITY_NONE
    };
    void* master_handler = NULL;

    esp_err_t err = mbc_master_init(MB_PORT_SERIAL_MASTER, &master_handler);
    MASTER_CHECK((master_handler != NULL), ESP_ERR_INVALID_STATE,
                                "mb controller initialization fail.");
    MASTER_CHECK((err == ESP_OK), ESP_ERR_INVALID_STATE,
                            "mb controller initialization fail, returns(0x%x).",
                            (uint32_t)err);
    err = mbc_master_setup((void*)&comm);
    MASTER_CHECK((err == ESP_OK), ESP_ERR_INVALID_STATE,
                            "mb controller setup fail, returns(0x%x).",
                            (uint32_t)err);

    // Set UART pin numbers
    err = uart_set_pin(mb_port, 17, 16, 0, 0);

    err = mbc_master_start();
    MASTER_CHECK((err == ESP_OK), ESP_ERR_INVALID_STATE,
                            "mb controller start fail, returns(0x%x).",
                            (uint32_t)err);

    MASTER_CHECK((err == ESP_OK), ESP_ERR_INVALID_STATE,
            "mb serial set pin failure, uart_set_pin() returned (0x%x).", (uint32_t)err);
    // Set driver mode to Half Duplex
    err = uart_set_mode(MB_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX);
    MASTER_CHECK((err == ESP_OK), ESP_ERR_INVALID_STATE,
            "mb serial set mode failure, uart_set_mode() returned (0x%x).", (uint32_t)err);

    vTaskDelay(5);
    err = mbc_master_set_descriptor(&device_parameters[0], num_device_parameters);
    MASTER_CHECK((err == ESP_OK), ESP_ERR_INVALID_STATE,
                                "mb controller set descriptor fail, returns(0x%x).",
                                (uint32_t)err);
    ESP_LOGI(MASTER_TAG, "Modbus master stack initialized...");
    return err;
}

void modbus_handle_task(void *arg){
    ESP_ERROR_CHECK(master_init());
    vTaskDelay(10);
    for(;;){
        if (!mwifi_is_connected() && !(mwifi_is_started() && esp_mesh_is_root())) {
            vTaskDelay(500 / portTICK_RATE_MS);
            continue;
        }
        master_operation_func(NULL);
    }
}
// void app_main(void)
// {
//     // Initialization of device peripheral and objects
//     ESP_ERROR_CHECK(master_init());
//     vTaskDelay(10);
    
//     master_operation_func(NULL);
// }
