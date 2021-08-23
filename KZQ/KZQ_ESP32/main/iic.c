/* i2c - Example

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "iic.h"
#include "mdf_common.h"
#define jcq_id sgp30
static esp_err_t i2c_master_init(void);
static const char *TAG = "i2c-example";

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 512                  /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH 128               /*!< Data length for r/w test, [0,DATA_LENGTH] */
#define DELAY_TIME_BETWEEN_ITEMS_MS 2000 /*!< delay time between different test items */

#define I2C_SLAVE_SCL_IO 12               /*!< gpio number for i2c slave clock */
#define I2C_SLAVE_SDA_IO 13               /*!< gpio number for i2c slave data */
#define I2C_SLAVE_NUM I2C_NUMBER(0) /*!< I2C port number for slave dev */
#define I2C_SLAVE_TX_BUF_LEN (2 * DATA_LENGTH)              /*!< I2C slave tx buffer size */
#define I2C_SLAVE_RX_BUF_LEN (2 * DATA_LENGTH)              /*!< I2C slave rx buffer size */

#define I2C_MASTER_SCL_IO 13               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 14               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(0) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 10000        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define LUX_V30_SENSOR_ADDR 0x4a   /*!< slave address for LUX_V30 sensor */
#define LUX_V30_CMD_START 0   /*!< Operation mode */
#define ESP_SLAVE_ADDR 0 /*!< ESP32 slave address, you can set any 7bit value */
#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */


#define SGP20_SENEOR_SDDR 0x58
#define SGP30_FEATURESET       0x0020  ///< The required set for this library
#define SGP30_CRC8_POLYNOMIAL  0x31    ///< Seed for SGP30's CRC polynomial
#define SGP30_CRC8_INIT        0xFF    ///< Init value for CRC
#define SGP30_WORD_LEN         2       ///< 2 bytes per word
/**
 * The last measurement of the IAQ-calculated Total Volatile Organic Compounds in ppb. This value is set when you call {@link IAQmeasure()}
 */
uint16_t TVOC;

/**
 * The last measurement of the IAQ-calculated equivalent CO2 in ppm. This value is set when you call {@link IAQmeasure()}
 */
uint16_t eCO2;

/**
 * The 48-bit serial number, this value is set when you call {@link begin()}
 */
uint16_t serialnumber[3];
SemaphoreHandle_t print_mux = NULL;

/**
 * @brief test code to read esp-i2c-slave
 *        We need to fill the buffer of esp slave device, then master can read them out.
 *
 * _______________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------------|--------------------|------|
 *
 */
static esp_err_t i2c_master_read_slave(i2c_port_t i2c_num, uint8_t *data_rd, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Test code to write esp-i2c-slave
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 *
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 *
 */
static esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t *data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief test code to operate on LUX_V30 sensor
 *
 * 1. set operation mode(e.g One time L-resolution mode)
 * _________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write 1 byte + ack  
 * --------|---------------------------|--------------------
 * 3. read data
 * ______________________________________________________________________________________
 * | start | slave_addr + rd_bit + ack | read 1 byte + ack  | read 1 byte + nack | stop |
 * --------|---------------------------|--------------------|--------------------|------|
 */

static esp_err_t i2c_master_sensor_test(i2c_port_t i2c_num, uint8_t *data)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, LUX_V30_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, LUX_V30_CMD_START, ACK_CHECK_EN);
    // i2c_master_write_byte(cmd, LUX_V30_CMD_START, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, LUX_V30_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data, ACK_VAL);
    i2c_master_read_byte(cmd, data+1, ACK_VAL);
    i2c_master_read_byte(cmd, data+2, ACK_VAL);
    i2c_master_read_byte(cmd, data+3, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
/**
 * @brief test code to operate on LUX_V30 sensor
 *
 * 1. set operation mode(e.g One time L-resolution mode)
 * _________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write 1 byte + ack  
 * --------|---------------------------|--------------------
 * 3. read data
 * ______________________________________________________________________________________
 * | start | slave_addr + rd_bit + ack | read 1 byte + ack  | read 1 byte + nack | stop |
 * --------|---------------------------|--------------------|--------------------|------|
 */

/*
command code :0x2003 初始化空气质量
*/
static esp_err_t IAQinit(void){
    int ret;
    uint8_t command[2];
    command[0] = 0x20;
    command[1] = 0x03;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SGP20_SENEOR_SDDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, command[0], ACK_CHECK_EN);
    i2c_master_write_byte(cmd, command[1], ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;   
}
/*
command code :0x2008 读取空气质量
*/
static esp_err_t IAQmeasure( uint8_t *eCO2, uint8_t *tVOC){
    int ret;
    uint8_t command[2];
    command[0] = 0x20;
    command[1] = 0x08;
    uint8_t crc1=0,crc2=0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SGP20_SENEOR_SDDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, command[0], ACK_CHECK_EN);
    i2c_master_write_byte(cmd, command[1], ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    cmd = i2c_cmd_link_create();
    vTaskDelay(10/portTICK_RATE_MS);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SGP20_SENEOR_SDDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, eCO2+1, ACK_VAL);
    i2c_master_read_byte(cmd, eCO2, ACK_VAL);
    i2c_master_read_byte(cmd, &crc1, ACK_VAL);
    i2c_master_read_byte(cmd, tVOC+1, ACK_VAL);
    i2c_master_read_byte(cmd, tVOC, ACK_VAL);
    i2c_master_read_byte(cmd, &crc2, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}


/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief i2c slave initialization
 */
static esp_err_t i2c_slave_init(void)
{
    int i2c_slave_port = I2C_SLAVE_NUM;
    i2c_config_t conf_slave;
    conf_slave.sda_io_num = I2C_SLAVE_SDA_IO;
    conf_slave.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf_slave.scl_io_num = I2C_SLAVE_SCL_IO;
    conf_slave.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf_slave.mode = I2C_MODE_SLAVE;
    conf_slave.slave.addr_10bit_en = 0;
    conf_slave.slave.slave_addr = ESP_SLAVE_ADDR;
    i2c_param_config(i2c_slave_port, &conf_slave);
    return i2c_driver_install(i2c_slave_port, conf_slave.mode, I2C_SLAVE_RX_BUF_LEN, I2C_SLAVE_TX_BUF_LEN, 0);
}

void i2c_test_task(void *arg)
{
    int i = 0;
    int ret=0;
    uint32_t task_idx = (uint32_t)arg;
    uint32_t sensor_data_h;
    int cnt = 0;
    MDF_ERROR_ASSERT(i2c_master_init());
#if jcq_id == sgp30
    IAQinit();
    vTaskDelay(1  / portTICK_RATE_MS);
#endif
    while (1) {
        MDF_LOGI( "TASK[%d] test cnt: %d", task_idx, cnt++);
#if jcq_id == sgp30
       ret=IAQmeasure(&eCO2,&TVOC);
        if (ret == ESP_ERR_TIMEOUT) {
            MDF_LOGE( "I2C Timeout");
        } else if (ret == ESP_OK) {
            MDF_LOGI("*******************\n");
            MDF_LOGI("TASK[%d]  MASTER READ SENSOR( SGP_V30 )\n", task_idx);
            MDF_LOGI("*******************\n");
            MDF_LOGI("TVOC: %d eCO2 %d\n",TVOC,eCO2);
            // MDF_LOGI("sensor val: %.02f [Lux]\n", (sensor_data_h << 8 | sensor_data_l) / 1.2);
        } else {
            MDF_LOGW( "%s: No ack, sensor not connected...skip...", mdf_err_to_name(ret));
        }
#elif jcq_id == blux30
        ret = i2c_master_sensor_test(I2C_MASTER_NUM, &sensor_data_h);
        if (ret == ESP_ERR_TIMEOUT) {
            MDF_LOGE( "I2C Timeout");
        } else if (ret == ESP_OK) {
            MDF_LOGI("*******************\n");
            MDF_LOGI("TASK[%d]  MASTER READ SENSOR( LUX_V30 )\n", task_idx);
            MDF_LOGI("*******************\n");
            MDF_LOGI("data_h: %d\n", sensor_data_h);
            // MDF_LOGI("sensor val: %.02f [Lux]\n", (sensor_data_h << 8 | sensor_data_l) / 1.2);
        } else {
            MDF_LOGW( "%s: No ack, sensor not connected...skip...", mdf_err_to_name(ret));
        }
#endif

        vTaskDelay(1000  / portTICK_RATE_MS);
    }
    vSemaphoreDelete(print_mux);
    vTaskDelete(NULL);
}
