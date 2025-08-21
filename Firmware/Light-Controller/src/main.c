#include "Externals.h"
#include "WebServer.h"

// temp
#include "personal_wifi_credentials.h"

void app_main() {
    Externals_Init();

    WebServer_Init(YOUR_WIFI_SSID, YOUR_WIFI_PASSWORD);
}