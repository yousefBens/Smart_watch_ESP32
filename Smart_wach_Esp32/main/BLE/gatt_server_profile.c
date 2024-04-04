/*
 * gatt_ser_gap.c
 *
 *  Created on: 7 mars 2023
 *      Author: herve
 */

#include "gatt_server_profile.h"

#define TEST_DEVICE_NAME            "ESP_GATT_SERV_Yousef"

extern struct gatts_profile_inst gl_profile_tab[PROFILE_NUM];

static uint8_t test_service_uuid128[32] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xAB, 0xCD, 0x00, 0x00,
    //second uuid, 32bit, [12], [13], [14], [15] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xAB, 0xCD, 0xAB, 0xCD,
};

//static uint8_t test_manufacturer[TEST_MANUFACTURER_DATA_LEN] =  {0x12, 0x23, 0x45, 0x56};

//Bluetooth GAP advertisement data
static esp_ble_adv_data_t test_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x20,
    .max_interval = 0x40,
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 32,
    .p_service_uuid = test_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

//Bluetooth GAP advertisement parameters
esp_ble_adv_params_t test_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

void gap_init(void) {
    ESP_LOGI(GATT_SERVER_TAG, "gap_init start");

    gl_profile_tab[PROFILE_APP_ID].service_id.is_primary = true;
    gl_profile_tab[PROFILE_APP_ID].service_id.id.inst_id = 0x00;
    gl_profile_tab[PROFILE_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
    gl_profile_tab[PROFILE_APP_ID].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID;

    ESP_LOGI(GATT_SERVER_TAG, "gap_init esp_ble_gap_set_device_name");
    esp_ble_gap_set_device_name(TEST_DEVICE_NAME);

    ESP_LOGI(GATT_SERVER_TAG, "gap_init esp_ble_gap_config_adv_data");
    esp_ble_gap_config_adv_data(&test_adv_data);

    esp_ble_gatts_create_service(gl_profile_tab[PROFILE_APP_ID].gatts_if, &gl_profile_tab[PROFILE_APP_ID].service_id, GATTS_NUM_HANDLE);

    ESP_LOGI(GATT_SERVER_TAG, "gap_init end");
}

void gap_start(void) {
    ESP_LOGI(GATT_SERVER_TAG, "gap_start %d", gl_profile_tab[PROFILE_APP_ID].gatts_if);
    esp_ble_gap_start_advertising(&test_adv_params);
}

void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    ESP_LOGI(GATT_SERVER_TAG, "gap_event_handler %d start", event);
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
    	gap_start();
        break;
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
    	gap_start();
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
    	gap_start();
        break;
    default:
        break;
    }
}
