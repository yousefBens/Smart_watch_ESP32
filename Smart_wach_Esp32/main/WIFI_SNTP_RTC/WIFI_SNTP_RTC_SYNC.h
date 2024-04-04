#ifndef WIFI_SNTP_RTC_SYNC
#define WIFI_SNTP_RTC_SYNC



#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "esp_netif_sntp.h"
#include "lwip/ip_addr.h"
#include "esp_sntp.h"
#include "sdkconfig.h"
#include "example_common_private.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "lwip/err.h"
#include "lwip/sys.h"


#ifndef INET6_ADDRSTRLEN
#define INET6_ADDRSTRLEN 48
#endif

/* Variable holding number of times ESP32 restarted since first boot.
 * It is placed into RTC memory using RTC_DATA_ATTR and
 * maintains its value when ESP32 wakes from deep sleep.
 */
extern TaskHandle_t Lecture_Horloge_Universelle_Handle;
void Lecture_Horloge_Universelle(void *pvParameters);
struct tm Lecture_RTC(void);
void Reglage_RTC(void);
void show_time(void);
void Test_de_Lecture_Heure_Universelle_et_Reglage_RTC(void); // Pour le rapport du projet




#endif // WIFI_SNTP_RTC_SYNC
