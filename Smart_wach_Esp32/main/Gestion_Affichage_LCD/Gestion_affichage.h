#ifndef GESTION_AFFICHAGE_H
#define GESTION_AFFICHAGE_H




#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_freertos_hooks.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "sdkconfig.h"
#include "driver/gptimer.h"
#include "esp_log.h"
#include "lvgl.h"
#include "lvgl_helpers.h"
#include "WIFI_SNTP_RTC_SYNC.h"
#include "lv_conf_internal.h"
#include "Gestion_affichage.h"
#include <time.h>





void init_screen_2_tableau_mesures(void);
void init_screen_3_graphe_temperature(void);
void init_screen_4_graphe_magnetometre(void);
void init_screen_5_graphe_gyroscope(void);
void init_screen_6_graphe_accelerometre(void);
void Affichage_LCD(void *args);
void init_screen_1_Affichage_Horodate(void);


























#endif // GESTION_AFFICHAGE_H
