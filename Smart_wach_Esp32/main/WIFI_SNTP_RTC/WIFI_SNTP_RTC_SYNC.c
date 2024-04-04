#include "WIFI_SNTP_RTC_SYNC.h"

static const char *TAG = "example";


#ifdef CONFIG_SNTP_TIME_SYNC_METHOD_CUSTOM
void sntp_sync_time(struct timeval *tv)
{
   settimeofday(tv, NULL);
   ESP_LOGI(TAG, "Time is synchronized from custom code");
   sntp_set_sync_status(SNTP_SYNC_STATUS_COMPLETED);
}
#endif

void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Notification of a time synchronization event");
}

static void obtain_time(void)
{
    ESP_ERROR_CHECK( nvs_flash_init() );
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK( esp_event_loop_create_default() );
#if LWIP_DHCP_GET_NTP_SRV
    /**
     * NTP server address could be acquired via DHCP,
     * see following menuconfig options:
     * 'LWIP_DHCP_GET_NTP_SRV' - enable STNP over DHCP
     * 'LWIP_SNTP_DEBUG' - enable debugging messages
     *
     * NOTE: This call should be made BEFORE esp acquires IP address from DHCP,
     * otherwise NTP option would be rejected by default.
     */
    ESP_LOGI(TAG, "Initializing SNTP");
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG(CONFIG_SNTP_TIME_SERVER);
    config.start = false;                       // start SNTP service explicitly (after connecting)
    config.server_from_dhcp = true;             // accept NTP offers from DHCP server, if any (need to enable *before* connecting)
    config.renew_servers_after_new_IP = true;   // let esp-netif update configured SNTP server(s) after receiving DHCP lease
    config.index_of_first_server = 1;           // updates from server num 1, leaving server 0 (from DHCP) intact
    // configure the event on which we renew servers
#ifdef CONFIG_EXAMPLE_CONNECT_WIFI
    config.ip_event_to_renew = IP_EVENT_STA_GOT_IP;
#elsez
    config.ip_event_to_renew = IP_EVENT_ETH_GOT_IP;
#endif
    config.sync_cb = time_sync_notification_cb; // only if we need the notification function
    esp_netif_sntp_init(&config);

#endif /* LWIP_DHCP_GET_NTP_SRV */

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    example_connect();
#if LWIP_DHCP_GET_NTP_SRV
    ESP_LOGI(TAG, "Starting SNTP");
    esp_netif_sntp_start();
#if LWIP_IPV6 && SNTP_MAX_SERVERS > 2
    /* This demonstrates using IPv6 address as an additional SNTP server
     * (statically assigned IPv6 address is also possible)
     */
    ip_addr_t ip6;
    if (ipaddr_aton("2a01:3f7::1", &ip6)) {    // ipv6 ntp source "ntp.netnod.se"
        esp_sntp_setserver(2, &ip6);
    }
#endif  /* LWIP_IPV6 */

#else
    ESP_LOGI(TAG, "Initializing and starting SNTP");
#if CONFIG_LWIP_SNTP_MAX_SERVERS > 1
    /* This demonstrates configuring more than one server
     */
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG_MULTIPLE(2,
                               ESP_SNTP_SERVER_LIST(CONFIG_SNTP_TIME_SERVER, "pool.ntp.org" ) );
#else
    /*
     * This is the basic default config with one server and starting the service
     */
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG(CONFIG_SNTP_TIME_SERVER);
#endif
    config.sync_cb = time_sync_notification_cb;     // Note: This is only needed if we want
#ifdef CONFIG_SNTP_TIME_SYNC_METHOD_SMOOTH
    config.smooth_sync = true;
#endif

    esp_netif_sntp_init(&config);
#endif

    int retry = 0;
    const int retry_count = 15;
    while (esp_netif_sntp_sync_wait(2000 / portTICK_PERIOD_MS) == ESP_ERR_TIMEOUT && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
    }
    if (retry < retry_count) {
        ESP_LOGI(TAG, "Synchronisation SNTP reussie ! ");
        Reglage_RTC();
    } else {
        ESP_LOGE(TAG, "Synchronisation SNTP echouee ! ");
    }

    ESP_ERROR_CHECK( example_disconnect() );
    esp_netif_sntp_deinit();
}





void Lecture_Horloge_Universelle(void *pvParameters) {
	while (1) {
		   obtain_time();
		   //Test_de_Lecture_Heure_Universelle_et_Reglage_RTC(); // Appel pour la fonction du test dans le cadre du rapport
		   vTaskSuspend(Lecture_Horloge_Universelle_Handle);
	   }
}


struct tm Lecture_RTC(void)
{
    time_t now;
    struct tm Horodate;
    time(&now);
    localtime_r(&now, &Horodate);
    return Horodate;
}

void Test_de_Lecture_Heure_Universelle_et_Reglage_RTC(void) {
    struct tm timeinfo = Lecture_RTC();
    char buffer[64];

    strftime(buffer, sizeof(buffer), "%Y-%m-%d - %H:%M:%S", &timeinfo);

  ESP_LOGI("Test Apres Lecture Heure Universelle et Reglage RTC", "\nHeure Actuelle apres Reglage_RTC"
		  "\n en utilisant la fonction Lecture_RTC :  Date - Heure : %s", buffer);
}



void Reglage_RTC(void) {
    setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1); /* Configuration pour le fuseau horaire français */
    tzset();
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    ESP_LOGI("RTC", "Heure mise à jour et selon le fuseau horaire !");
}

void show_time(void) { /* You can use this task for Clock testing purpose */
	       time_t time_now;
	       struct tm timeinfo;
	       time(&time_now);
	       localtime_r(&time_now, &timeinfo);
	       ESP_LOGI("Smart Watch Display", "Screen 1 RTC display : %d-%d-%d %d:%d:%d",
	                timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
	                timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

}




