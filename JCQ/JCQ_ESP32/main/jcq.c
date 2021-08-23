// Copyright 2017 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>
#include "sdkconfig.h"

#include "mdf_common.h"
#include "mwifi.h"
#include "driver/uart.h"
#include "string.h"

#include "uart.h"
#include "socket.h"
#include "mesh.h"
#include "iic.h"
#include "driver/timer.h"
#include "dht11.h"
#include "gatts.h"
#include "spiffs.h"
#include "master.h"
static const char *TAG = "JCQ";
// #define MEMORY_DEBUG
char zp13_val[20] = {0};
#define I_PIN 12
static void gpio_handle_task(void *arg)
{
    gpio_pad_select_gpio(I_PIN);
    gpio_set_direction(I_PIN, GPIO_MODE_INPUT);
    uint8_t read_val = 0;
    while (1)
    {
        read_val = gpio_get_level(I_PIN);
        memset(zp13_val, 0, sizeof(zp13_val));
        sprintf(zp13_val, "%d", read_val);
        vTaskDelay(500 / portTICK_RATE_MS);
    }
}
#define GPIO_KEY_NUM 0

/* 定义LED闪烁定时器句柄*/
TimerHandle_t LED_Timer_Handle;
/* 声明定时器回调函数 */
void LED_Timer_Callback(TimerHandle_t xTimer);

/* 定义key定时器句柄*/
TimerHandle_t Key_Timer_Handle;
/* 声明key定时器回调函数 */
void KEY_Timer_Callback(TimerHandle_t xTimer);

/* 定义一个按键值消息队列句柄 */
QueueHandle_t Key_Queue;

/* gpio中断处理函数*/
static void gpio_isr_handler(void *arg)
{
    xTimerResetFromISR(Key_Timer_Handle, NULL);
}

void key_init()
{

    /* 定义一个gpio配置结构体 */
    gpio_config_t gpio_config_structure;

    // /* 初始化gpio配置结构体*/
    // gpio_config_structure.pin_bit_mask = (1ULL << GPIO_LED_NUM);/* 选择gpio2 */
    // gpio_config_structure.mode = GPIO_MODE_OUTPUT;              /* 输出模式 */
    // gpio_config_structure.pull_up_en = 0;                       /* 不上拉 */
    // gpio_config_structure.pull_down_en = 0;                     /* 不下拉 */
    // gpio_config_structure.intr_type = GPIO_PIN_INTR_DISABLE;    /* 禁止中断 */

    // /* 根据设定参数初始化并使能 */
    // gpio_config(&gpio_config_structure);

    /* 初始化gpio配置结构体*/
    gpio_config_structure.pin_bit_mask = (1ULL << GPIO_KEY_NUM); /* 选择gpio0 */
    gpio_config_structure.mode = GPIO_MODE_INPUT;                /* 输入模式 */
    gpio_config_structure.pull_up_en = 0;                        /* 不上拉 */
    gpio_config_structure.pull_down_en = 0;                      /* 不下拉 */
    gpio_config_structure.intr_type = GPIO_PIN_INTR_NEGEDGE;     /* 下降沿触发中断 */

    /* 根据设定参数初始化并使能 */
    gpio_config(&gpio_config_structure);

    /* 开启gpio中断服务 */
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1); /* LEVEL1为最低优先级 */
    /* 设置GPIO的中断回调函数 */
    gpio_isr_handler_add(GPIO_KEY_NUM, gpio_isr_handler, (void *)GPIO_KEY_NUM);

    // LED_Timer_Handle = xTimerCreate((const char*)"LED Timer",                   /* 软件定时器名称 */
    // 					            (TickType_t )(1000 / portTICK_PERIOD_MS),   /* 定时周期，单位为时钟节拍 */
    // 						        (UBaseType_t)pdTRUE,                        /* 定时器模式，是否为周期定时模式 */
    // 						        (void*)1,                                   /* 定时器ID号 */
    // 						        (TimerCallbackFunction_t)LED_Timer_Callback);/*定时器回调函数 */
    // if((LED_Timer_Handle != NULL))
    //     ret = xTimerStart(LED_Timer_Handle,0);	    /* 创建成功，开启定时器*/
    // else
    //     printf("LED Timer Create failure !!! \n");  /* 定时器创建失败 */

    // if(ret == pdPASS)
    //     printf("LED Timer Start OK. \n");           /* 定时器启动成功*/
    // else
    //     printf("LED Timer Start err. \n");          /* 定时器启动失败*/

    /* 创建按键检测定时器，50ms,单次定时*/
    Key_Timer_Handle = xTimerCreate("Key Timer", (50 / portTICK_PERIOD_MS), pdFALSE, (void *)1, KEY_Timer_Callback);

    /* 创建按键检测消息队列 */
    Key_Queue = xQueueCreate((UBaseType_t)10,  /* 队列长度，这里是队列的项目数，即这个队列可以接受多少个消息*/
                             (UBaseType_t)50); /* 队列中每个消息的长度，单位为字节 */
}
void KEY_Timer_Callback(TimerHandle_t xTimer)
{
    if (!gpio_get_level(0))
    {
        static int key_times = 0;
        char msg[50];
        key_times++;
        printf("BOOT KEY have pressed. \n");
        sprintf(msg, "BOOT KEY have pressed %d times.", key_times);
        xQueueSendFromISR(Key_Queue, msg, NULL);
    }
}
#define led_pin 2
static void led_handle_task(void *arg)
{
    gpio_pad_select_gpio(led_pin);
    gpio_set_direction(led_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(led_pin, 0);
    char led_state = 0;
    while (1)
    {
        led_state = !led_state;
        gpio_set_level(led_pin, led_state);
        vTaskDelay(500 / portTICK_RATE_MS);
    }
}
static void command_handle_task(void *arg)
{
    char command_state = 0;
    key_init();
    char msg[50] = {0};
    while (1)
    {
        if (xQueueReceive(Key_Queue, msg, portMAX_DELAY))
        {
            MDF_LOGI("COMMAND STATE: %s", msg);
            command_state = !command_state;
            if (command_state)
            {
                if (mwifi_is_started())
                {
                    mwifi_stop();
                    esp_wifi_stop();
                }
                BLE_INIT();
            }
            else
            {
                BLE_DEINIT();
                esp_wifi_start();
                mwifi_start();
            }
        }
    }
}
char temp_val[20] = {0}, hum_val[20] = {0};
static void dht11_handle_task(void *arg)
{
    vTaskDelay(3000 / portTICK_RATE_MS);
    if (!DHT11_Init())
    {
        MDF_LOGI("DH11 checked ok.\n");
    }
    else
    {
        MDF_LOGI("DH11 checked failed.\n");
    }
    while (1)
    {
        uint8_t temp = 0, hum = 0;
        DHT11_Read_Data(&temp, &hum);
        if (hum > 0)
        {
            MDF_LOGI("DH11 T:%d, H: %d.\n", temp, hum);
            memset(temp_val, 0, sizeof(temp_val));
            memset(hum_val, 0, sizeof(hum_val));
            sprintf(temp_val, "%d", temp);
            sprintf(hum_val, "%d", hum);
        }
        vTaskDelay(2000 / portTICK_RATE_MS);
    }
}

void app_main()
{
    spi_ffs();
    wifi_mesh_init();
    //BLE_INIT();
    xTaskCreate(led_handle_task, "led_handle_task", 4 * 1024,
                NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);
    xTaskCreate(command_handle_task, "command_handle_task", 4 * 1024,
                NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);
    xTaskCreate(gpio_handle_task, "gpio_handle_task", 4 * 1024,
                NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);
    xTaskCreate(uart_handle_task, "uart_handle_task", 4 * 1024,
                NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);
    xTaskCreate(uart1_handle_task, "uart1_handle_task", 4 * 1024,
                NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);
    xTaskCreate(modbus_handle_task, "modbus_handle_task", 4 * 1024,
                NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);
    xTaskCreate(i2c_test_task, "i2c_test_task", 1024 * 2, (void *)0, 10, NULL);
    xTaskCreate(i2c1_test_task, "i2c_test_task", 1024 * 2, (void *)0, 10, NULL);
    xTaskCreate(dht11_handle_task, "dht11_handle_task", 1024 * 2, (void *)0, 10, NULL);
}