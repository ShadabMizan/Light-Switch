#include "Externals.h"
#include "WebServer.h"
#include "WiFiConfig.h"

void app_main(void) {
    Externals_Init();

    wifi_config_data_t config;
    if (WiFiConfig_Load(&config) != ESP_OK) {
        WiFiConfig_StartPortal();
    } else {
        BackEnd_Init(config.ssid, config.password);
    }
}
