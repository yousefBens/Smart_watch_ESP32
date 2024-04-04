/*
 * gatt_profile.h
 *
 *  Created on: 13 mars 2022
 *      Author: herve
 */
#pragma once

#ifndef MAIN_GATT_SERVER_PROFILE_H_
#define MAIN_GATT_SERVER_PROFILE_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sdkconfig.h"

#include "esp_system.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"

void gap_init(void);

void gap_start(void);

void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
//void gatts_profile_event_handler_2(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

//typedef struct blufi_beacon_s blufi_beacon_t;

#define PROFILE_NUM 1
#define PROFILE_APP_ID 0

#define GATT_SERVER_TAG "GATT_SERVER"

#define GATTS_SERVICE_UUID   0xFF10
#define GATTS_SERVICE2_UUID   0xFF11
#define GATTS_CHAR_UUID      0xFF12
#define GATTS_CHAR2_UUID      0xFF13
#define GATTS_DESCR_UUID     0x3333
#define GATTS_NUM_HANDLE     4

//Bluetooth GATT profile instance structure
struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;

    esp_bt_uuid_t char_uuid;

    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;

};

#pragma once
/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */

#endif /* MAIN_GATT_SERVER_PROFILE_H_ */
