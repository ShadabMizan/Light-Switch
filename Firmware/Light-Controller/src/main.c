#include "Externals.h"
#include "WebServer.h"
#include "Credentials.h"

void app_main(void) {
    Externals_Init();
    
    BackEnd_Init(WIFI_SSID, WIFI_PASSWORD);
}
