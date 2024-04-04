/*
 * gatt_profile.c
 *
 *  Created on: 7 mars 2023
 *      Author: herve
 */

#include "gatt_server_profile.h"

#define GATTS_DEMO_CHAR_VAL_LEN_MAX		0x80

extern char Temperature_buf[20];
extern char Acc_buf[100];
extern char Ang_buf[100];
extern char Mag_buf[100];
extern esp_ble_adv_params_t test_adv_params;

esp_attr_value_t gatts_demo_char1_val =
{
        .attr_max_len = GATTS_DEMO_CHAR_VAL_LEN_MAX,
	.attr_len		= sizeof(Acc_buf),
	.attr_value     = (uint8_t *)Acc_buf,
};

struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    ESP_LOGI(GATT_SERVER_TAG, "gatts_profile_event_handler %d %d", event,gatts_if);
    switch (event) {

    case ESP_GATTS_REG_EVT:
        ESP_LOGI(GATT_SERVER_TAG, "REGISTER_APP_EVT, status %d, app_id %d", param->reg.status, param->reg.app_id);
        gap_init();
        break;

    case ESP_GATTS_READ_EVT: {

    	ESP_LOGI(GATT_SERVER_TAG, "GATT_READ_EVT, conn_id %d, trans_id %lu, handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);

    	esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));

        //Here we fill the rsp (response) structure with our data (i.e. the temperature value).
        rsp.attr_value.handle = param->read.handle;
        rsp.attr_value.len = 154;

        for(int i = 0; i<10; i++){
        	rsp.attr_value.value[i] = Temperature_buf[i];
        }

        for(int i = 10; i<58; i++){
        	rsp.attr_value.value[i] = Acc_buf[i-10];
        }


        for(int i = 58; i<106; i++){
        	rsp.attr_value.value[i] = Ang_buf[i-58];
        }


        for(int i = 106; i<154; i++){
        	rsp.attr_value.value[i] = Mag_buf[i-106];
        }


        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                    ESP_GATT_OK, &rsp);
        break;
    }
    case ESP_GATTS_WRITE_EVT: {

        ESP_LOGI(GATT_SERVER_TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %lu, handle %d\n", param->write.conn_id, param->write.trans_id, param->write.handle);
        ESP_LOGI(GATT_SERVER_TAG, "GATT_WRITE_EVT, value len %d, value %08lx\n", param->write.len, *(uint32_t *)param->write.value);

        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);

        static char buf[128]; // do not free from heap!
        bzero(buf,sizeof(buf));
		memcpy(buf, param->write.value, param->write.len);
        ESP_LOGI(GATT_SERVER_TAG, "GATT_WRITE_EVT, value %s\n", buf);

        //You can add your code here to handle a specific task when the WRITE EVENT occurs

        break;
    }
    case ESP_GATTS_EXEC_WRITE_EVT:
    case ESP_GATTS_MTU_EVT:
    case ESP_GATTS_CONF_EVT:
    case ESP_GATTS_UNREG_EVT:
        break;

    case ESP_GATTS_CREATE_EVT:

        ESP_LOGI(GATT_SERVER_TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d\n", param->create.status, param->create.service_handle);

        gl_profile_tab[PROFILE_APP_ID].service_handle = param->create.service_handle;
        gl_profile_tab[PROFILE_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_APP_ID].char_uuid.uuid.uuid16 = GATTS_CHAR_UUID;

        esp_ble_gatts_start_service(gl_profile_tab[PROFILE_APP_ID].service_handle);

        esp_ble_gatts_add_char(gl_profile_tab[PROFILE_APP_ID].service_handle,
        		&gl_profile_tab[PROFILE_APP_ID].char_uuid,
				ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
				ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                &gatts_demo_char1_val,
				NULL);
        break;
    case ESP_GATTS_ADD_INCL_SRVC_EVT:
        break;
    case ESP_GATTS_ADD_CHAR_EVT: {
	    uint16_t length = 0;
        const uint8_t *prf_char;

        ESP_LOGI(GATT_SERVER_TAG, "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d\n",
                param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
        gl_profile_tab[PROFILE_APP_ID].char_handle = param->add_char.attr_handle;
        gl_profile_tab[PROFILE_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        esp_ble_gatts_get_attr_value(param->add_char.attr_handle,  &length, &prf_char);

        ESP_LOGI(GATT_SERVER_TAG, "the gatts demo char length = %x\n", length);
        for(int i = 0; i < length; i++){
            ESP_LOGI(GATT_SERVER_TAG, "prf_char[%x] =%x\n",i,prf_char[i]);
        }
        esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_APP_ID].service_handle, &gl_profile_tab[PROFILE_APP_ID].descr_uuid,
                                     ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
        break;
    }
    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        ESP_LOGI(GATT_SERVER_TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d\n",
                 param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
        break;
    case ESP_GATTS_DELETE_EVT:
        break;
    case ESP_GATTS_START_EVT:
        ESP_LOGI(GATT_SERVER_TAG, "SERVICE_START_EVT, status %d, service_handle %d\n",
                 param->start.status, param->start.service_handle);
        break;
    case ESP_GATTS_STOP_EVT:
        break;
    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(GATT_SERVER_TAG, "SERVICE_START_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:\n",
                 param->connect.conn_id,
                 param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                 param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
        gl_profile_tab[PROFILE_APP_ID].conn_id = param->connect.conn_id;
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        esp_ble_gap_start_advertising(&test_adv_params);
        break;
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    default:
        break;
    }
    ESP_LOGI(GATT_SERVER_TAG, "gatts_profile_event_handler end");
}



void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    ESP_LOGI(GATT_SERVER_TAG, "gatts_event_handler %d %d start", event,gatts_if);
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            ESP_LOGI(GATT_SERVER_TAG, "Reg app ok, app_id %04x, status %d\n", param->reg.app_id, param->reg.status);
            gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
        } else {
            ESP_LOGI(GATT_SERVER_TAG, "Reg app failed, app_id %04x, status %d\n",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    gatts_profile_event_handler(event, gatts_if, param);
    ESP_LOGI(GATT_SERVER_TAG, "gatts_event_handler end");
}





