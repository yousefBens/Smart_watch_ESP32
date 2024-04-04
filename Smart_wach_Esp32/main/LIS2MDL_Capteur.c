







#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "lis2mdl_reg.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "esp32_lis2mdl";

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DELAY_TIME_BETWEEN_ITEMS_MS 1000 /*!< delay time between different test items */

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

//
#define LIS2MDL_ADDR_7BITS 0x1e
//
#define SENSOR_BUS I2C_MASTER_NUM
#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

typedef union{
  int16_t i16bit;
  uint8_t u8bit[2];
} axis1bit16_t;

//static lsm6dso_id_t whoamI;
static uint8_t whoamI;
stmdev_ctx_t dev_ctxx;

static int16_t data_raw_magnetic[3];
static float magnetic_mG[3];

TaskHandle_t xHandle = NULL;

extern int a;

struct ValueToSend_Task {
  float x;
  float y;
  float z;
};

extern char Mag_buf[100];


struct ValueToSend_Task Mag_values;

extern QueueHandle_t xQueue_Mag;
/*

static axis1bit16_t data_raw_temperature;
static float temperature_degC;
TaskHandle_t xHandle = NULL;*/


/**
 * @brief test code to read esp-i2c-slave like a memory device
 *        We need to fill the buffer of esp slave device, then master can read them out.
 *
 * _____________________________________________________________________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write 1 byte (register address) + ack | start | slave_addr + rd_bit + ack | read n bytes + nack | stop |
 * --------|---------------------------|---------------------------------------|-------|---------------------------|---------------------|------|
 *
 */
static int32_t i2c_master_read_slave(uint8_t i2c_num, uint8_t regaddr, uint8_t *data_rd, uint16_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LIS2MDL_ADDR_7BITS << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, regaddr, ACK_CHECK_EN);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LIS2MDL_ADDR_7BITS << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}


/**
 * @brief Test code to write esp-i2c-slave
 *        Master device write data to slave memory like device,
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 *
 * __________________________________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write 1 byte (register address) + ack | write n bytes + ack  | stop |
 * --------|---------------------------|---------------------------------------|----------------------|------|
 *
 */
static int32_t i2c_master_write_slave(uint8_t i2c_num, uint8_t regaddr, uint8_t *data_wr, uint16_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LIS2MDL_ADDR_7BITS << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, regaddr, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief i2c master initialization
 */
/*static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;

    conf.clk_flags = 0;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE; // Pullup resistors are already present on X-NUCLEO-IKS01A3
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}*/

static void get_mag_task(void *args)
{
	/* Read output only if not busy */
	uint8_t flag;

 while (1) {
	 lis2mdl_mag_data_ready_get(&dev_ctxx, &flag);
	if (flag)
	{
	  /* Read temperature data */
		memset(data_raw_magnetic, 0x00, 3 * sizeof(int16_t));
		lis2mdl_magnetic_raw_get(&dev_ctxx, data_raw_magnetic);
		magnetic_mG[0] = lis2mdl_from_lsb_to_mgauss(data_raw_magnetic[0]);
		magnetic_mG[1] = lis2mdl_from_lsb_to_mgauss(data_raw_magnetic[1]);
		magnetic_mG[2] = lis2mdl_from_lsb_to_mgauss(data_raw_magnetic[2]);
		printf("Magnetic field [mG]:%4.2f\t%4.2f\t%4.2f\r\n", magnetic_mG[0], magnetic_mG[1], magnetic_mG[2]);

	      Mag_values.x = magnetic_mG[0];
	      Mag_values.y = magnetic_mG[1];
	      Mag_values.z = magnetic_mG[2];

	      xQueueSend(xQueue_Mag, &Mag_values, 150); // Envoi de la valeur dans la file d'attente

	      sprintf(Mag_buf,"|MagX %f|MagY %f|MagZ %f\r\n", magnetic_mG[0], magnetic_mG[1], magnetic_mG[2]);



	}
	//vTaskDelay(2000 / portTICK_PERIOD_MS);
   }
}


void LIS2MDL(){
    //ESP_ERROR_CHECK(i2c_master_init());

    /* This acts as the entry point of ST's STTS751 driver */
    dev_ctxx.write_reg = i2c_master_write_slave;
    dev_ctxx.read_reg = i2c_master_read_slave;
    dev_ctxx.i2c_number = SENSOR_BUS;

    /* Enable interrupt on high(=49.5 degC)/low(=-4.5 degC) temperature. */
    /*float temperature_high_limit = 49.5f;
    stts751_high_temperature_threshold_set(&dev_ctx, stts751_from_celsius_to_lsb(temperature_high_limit));

    float temperature_low_limit = -4.5f;
    stts751_low_temperature_threshold_set(&dev_ctx, stts751_from_celsius_to_lsb(temperature_low_limit));*/
    /* Enable Block Data Update */
    lis2mdl_block_data_update_set(&dev_ctxx, PROPERTY_ENABLE);
    /* Set Output Data Rate */
    lis2mdl_data_rate_set(&dev_ctxx, LIS2MDL_ODR_10Hz);
    /* Set / Reset sensor mode */
    lis2mdl_set_rst_mode_set(&dev_ctxx, LIS2MDL_SENS_OFF_CANC_EVERY_ODR);
    /* Enable temperature compensation */
    lis2mdl_offset_temp_comp_set(&dev_ctxx, PROPERTY_ENABLE);
    /* Set device in continuous mode */
    lis2mdl_operating_mode_set(&dev_ctxx, LIS2MDL_CONTINUOUS_MODE);
    /* Configure filtering chain(No aux interface)
     * Accelerometer - LPF1 + LPF2 path
     */
}


void whoami_LIS2MDL_task()
{
	LIS2MDL();
    int ret;

    while (1) {
        ESP_LOGI(TAG, "WHOAMI TASK");

        lis2mdl_device_id_get(&dev_ctxx, &whoamI);
        printf("WhoamI = %d  \n", whoamI);
        if ( whoamI != LIS2MDL_ID )
        	ret = ESP_ERR_TIMEOUT; /* manage here device not found */
        else ret = ESP_OK;

        if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "I2C Timeout");
        } else if (ret == ESP_OK) {
            printf("************************************\n");
            printf("WHOAMI MASTER CHECK SENSOR (lis2mdl)\n");
            printf("************************************\n");
            /*printf("Product ID= %d \n", whoamI.product_id);
            printf("Manufacturer ID= %d \n", whoamI.manufacturer_id);
            printf("Revision ID= %d \n", whoamI.revision_id);*/
        } else {
            ESP_LOGW(TAG, "%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));
        }

       printf("End of Task WHOAMI!\n\n");

       /* whoami task is run only once. At the end we start get_temp_task n*/
       xTaskCreate(get_mag_task, "lis2mdl_get_Mag_task", 1024 * 2, (void *)0, 3, NULL);

       vTaskDelete(xHandle);
  }
}
