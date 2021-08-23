#include "mdf_common.h"
#include "mwifi.h"
#include "driver/uart.h"
#include "string.h"

#include "uart.h"
#include "socket.h"
#include "mesh.h"
#include "spiffs.h"
#include "gatts.h"
// #define MEMORY_DEBUG

static const char *TAG = "ZNWG";

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

static void led_handle_task(void *arg)
{
    gpio_pad_select_gpio(12);
    gpio_set_direction(12, GPIO_MODE_OUTPUT);
    gpio_set_level(12, 0);
    char led_state = 0;
    while (1)
    {
        led_state = !led_state;
        gpio_set_level(12, led_state);
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
void app_main()
{
    spi_ffs();
    xTaskCreate(led_handle_task, "led_handle_task", 1024,
                NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);
    xTaskCreate(command_handle_task, "command_handle_task", 2 * 1024,
                NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);
    /**
     * @brief uart handle task:
     *  receive json format data,eg:`{"dest_addr":"30:ae:a4:80:4c:3c","data":"send data"}`
     *  forward data item to destination address in mesh network
     */
    xTaskCreate(uart_handle_task, "uart_handle_task", 4 * 1024,
                NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);
}
