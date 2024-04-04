/*
 * ble_init.c
 *
 *  Created on: 14 mars 2022
 *      Author: herve
 */
#include "ble_init.h"
#include "esp_bt_main.h"

void ble_init(void) {
	printf("init BLE init\r\n");
esp_err_t ret;

// Initialize NVS.
ret = nvs_flash_init();
if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
	ESP_ERROR_CHECK(nvs_flash_erase());
	ret = nvs_flash_init();
}
ESP_ERROR_CHECK( ret );

ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
ret = esp_bt_controller_init(&bt_cfg);
if (ret) {
	ESP_LOGE(GATT_SERVER_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
	return;
}

ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
if (ret) {
	ESP_LOGE(GATT_SERVER_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
	return;
}
ret = esp_bluedroid_init();
if (ret) {
	ESP_LOGE(GATT_SERVER_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
	return;
}
ret = esp_bluedroid_enable();
if (ret) {
	ESP_LOGE(GATT_SERVER_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
	return;
}

printf("Fin init BLE init\r\n");

}
