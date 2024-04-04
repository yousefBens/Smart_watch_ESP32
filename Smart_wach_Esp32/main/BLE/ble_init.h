/*
 * ble_init.h
 *
 *  Created on: 14 mars 2022
 *      Author: herve
 */

#ifndef MAIN_BLE_INIT_H_
#define MAIN_BLE_INIT_H_

#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#include "gatt_server_profile.h"

void ble_init();

#endif /* MAIN_BLE_INIT_H_ */
