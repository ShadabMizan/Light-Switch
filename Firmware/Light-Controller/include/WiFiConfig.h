#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

#include "esp_err.h"

typedef struct {
    char ssid[32];
    char password[64];
    char device_name[32];
} wifi_config_data_t;

esp_err_t WiFiConfig_StartPortal(void);
esp_err_t WiFiConfig_Load(wifi_config_data_t *config);

#endif
