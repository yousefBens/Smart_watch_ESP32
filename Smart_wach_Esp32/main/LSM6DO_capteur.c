








#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_freertos_hooks.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "sdkconfig.h"

#include "lsm6dso_reg.h"
#include "driver/i2c.h"

//******************************************************************
//**************************      1. USER INCLUDES     *************
//******************************************************************
/* Additional peripherals and tools */
#include "driver/gptimer.h"  // replace driver/timer.h (since 2023)
#include "esp_log.h"		//  for debug fct :ESP_LOGI/W/E/D/V( )

/* Littlevgl specific */
#include "lvgl.h"
#include "lvgl_helpers.h"


#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DELAY_TIME_BETWEEN_ITEMS_MS 1000 /*!< delay time between different test items */

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */


#define LSM6DS0_ADDR_7BITS 0x6b
#define SENSOR_BUS I2C_MASTER_NUM
#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

//static lsm6dso_id_t whoamI;
static uint8_t whoamI;
stmdev_ctx_t dev_ctx;
static uint16_t data_raw_acceleration[3];
static uint16_t data_raw_angular_rate[3];
static int16_t data_raw_temperature;
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static float temperature_degC;
TaskHandle_t whoami_task_Handle = NULL;
TaskHandle_t acc_task_Handle = NULL;

int a = 1;

extern char Temperature_buf[20];
extern char Acc_buf[100];
extern char Ang_buf[100];

struct ValueToSend_Task {
  float x;
  float y;
  float z;
};


struct ValueToSend_Task Acc_values;
struct ValueToSend_Task Ang_values;

extern QueueHandle_t xQueue_T;
extern QueueHandle_t xQueue_Acc;
extern QueueHandle_t xQueue_Ang;
//extern SemaphoreHandle_t Ac_semaphore;

/**
 * @brief test code to read esp-i2c-slave like a memory device
 *        We need to fill the buffer of esp slave device, then master can read them out.
 *
 * _____________________________________________________________________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write 1 byte (register address) + ack | start | slave_addr + rd_bit + ack | read n bytes + nack | stop |
 * --------|---------------------------|---------------------------------------| -------|---------------------------|---------------------|------|
 *
 */
static int32_t i2c_master_read_slave(uint8_t i2c_num, uint8_t regaddr, uint8_t *data_rd, uint16_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM6DS0_ADDR_7BITS << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, regaddr, ACK_CHECK_EN);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM6DS0_ADDR_7BITS << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
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
    i2c_master_write_byte(cmd, (LSM6DS0_ADDR_7BITS << 1) | WRITE_BIT, ACK_CHECK_EN);
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

static void get_accel_and_temp_task()
{
	/* Read output only if not busy */

	uint8_t flag;

 while (1) {
	 lsm6dso_xl_flag_data_ready_get(&dev_ctx, &flag);
	if (flag)
	{
		memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
	  lsm6dso_acceleration_raw_get(&dev_ctx, &data_raw_acceleration);
      acceleration_mg[0] =
        lsm6dso_from_fs2_to_mg(data_raw_acceleration[0]);
      acceleration_mg[1] =
        lsm6dso_from_fs2_to_mg(data_raw_acceleration[1]);
      acceleration_mg[2] =
        lsm6dso_from_fs2_to_mg(data_raw_acceleration[2]);


      printf("Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
    		  acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);



      Acc_values.x = acceleration_mg[0];
      Acc_values.y = acceleration_mg[1];
      Acc_values.z = acceleration_mg[2];

      xQueueSend(xQueue_Acc, &Acc_values, 150); // Envoi de la valeur dans la file d'attente

      sprintf(Acc_buf,"|AccX %f|AccY %f|AccZ %f\r\n", acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
	}

      lsm6dso_gy_flag_data_ready_get(&dev_ctx, &flag);

      if (flag) {
        // Read angular rate field data
        memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
        lsm6dso_angular_rate_raw_get(&dev_ctx, &data_raw_angular_rate);
        angular_rate_mdps[0] =
          lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate[0]);
        angular_rate_mdps[1] =
          lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate[1]);
        angular_rate_mdps[2] =
          lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate[2]);
        printf("Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
                angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);

        Ang_values.x = angular_rate_mdps[0];
        Ang_values.y = angular_rate_mdps[1];
        Ang_values.z = angular_rate_mdps[2];

        xQueueSend(xQueue_Ang, &Ang_values, 150); // Envoi de la valeur dans la file d'attente
        sprintf(Ang_buf,"|AngX %f|AngY %f|AngZ %f\r\n", angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);

      }




      lsm6dso_temp_flag_data_ready_get(&dev_ctx, &flag);

      if (flag) {
        //Read temperature data
        memset(&data_raw_temperature, 0x00, sizeof(int16_t));
        lsm6dso_temperature_raw_get(&dev_ctx, &data_raw_temperature);
        temperature_degC =
          lsm6dso_from_lsb_to_celsius(data_raw_temperature);
        printf("Temperature [degC]:%6.2f\r\n", temperature_degC);

        xQueueSend(xQueue_T, &temperature_degC, 150); // Envoi de la valeur dans la file d'attente

        sprintf(Temperature_buf,"|T %f\r\n", temperature_degC);

      }
      //vTaskDelay(2000 / portTICK_PERIOD_MS);
      //xSemaphoreGive(Ac_semaphore);



   }

}


void LSM6DO(){
	//ESP_ERROR_CHECK(i2c_master_init());

	/* This acts as the entry point of ST's STTS751 driver */
	dev_ctx.write_reg = i2c_master_write_slave;
	dev_ctx.read_reg = i2c_master_read_slave;
	dev_ctx.i2c_number = SENSOR_BUS;

	/* Disable I3C interface */
	lsm6dso_i3c_disable_set(&dev_ctx, LSM6DSO_I3C_DISABLE);
	/* Enable Block Data Update */
	lsm6dso_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
	/* Set Output Data Rate */
	lsm6dso_xl_data_rate_set(&dev_ctx, LSM6DSO_XL_ODR_12Hz5);
	lsm6dso_gy_data_rate_set(&dev_ctx, LSM6DSO_GY_ODR_12Hz5);
	/* Set full scale */
	lsm6dso_xl_full_scale_set(&dev_ctx, LSM6DSO_2g);
	lsm6dso_gy_full_scale_set(&dev_ctx, LSM6DSO_2000dps);
	/* Configure filtering chain(No aux interface)
	* Accelerometer - LPF1 + LPF2 path
	*/
	lsm6dso_xl_hp_path_on_out_set(&dev_ctx, LSM6DSO_LP_ODR_DIV_100);
	lsm6dso_xl_filter_lp2_set(&dev_ctx, PROPERTY_ENABLE);

	//xTaskCreate(whoami_LSM6DO_task, "i2c_whoami_task", 1024 * 2, (void *)0, 10, &whoami_task_Handle);


}


void whoami_LSM6DO_task()
{
	LSM6DO();
    int ret;

    //while (1) {
        //ESP_LOGI(TAG, "WHOAMI TASK");

        lsm6dso_device_id_get(&dev_ctx, &whoamI);
        printf("WhoamI = %d  \n", whoamI);
        if ( whoamI != LSM6DSO_ID )
        	ret = ESP_ERR_TIMEOUT; /* manage here device not found */
        else ret = ESP_OK;

        if (ret == ESP_ERR_TIMEOUT) {
            //ESP_LOGE(TAG, "I2C Timeout");
        } else if (ret == ESP_OK) {
            printf("************************************\n");
            printf("WHOAMI MASTER CHECK SENSOR (lsm6dso)\n");
            printf("************************************\n");
            /*printf("Product ID= %d \n", whoamI.product_id);
            printf("Manufacturer ID= %d \n", whoamI.manufacturer_id);
            printf("Revision ID= %d \n", whoamI.revision_id);*/
        } else {
            //ESP_LOGW(TAG, "%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));
        }

       printf("End of Task WHOAMI!\n\n");

       /* whoami task is run only once. At the end we start get_temp_task n*/
       //get_accel_and_temp_task();
       //xTaskCreate(get_accel_and_temp_task, "lsm6dso_get_Acc_task", 10000, NULL, 1, &acc_task_Handle);
       //get_accel_and_temp_task();
       xTaskCreate(get_accel_and_temp_task, "lsm6dso_get_Acc_task", 1024 * 2, (void *)0, 3, NULL);
       //get_accel_and_temp_task();
       vTaskDelete(whoami_task_Handle);
  //}//
}




