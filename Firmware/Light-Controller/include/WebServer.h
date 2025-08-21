#ifndef WEBSERVER_H
#define WEBSERVER_H

void WebServer_Init(const char *ssid, const char *password);

void Update_PowerDelivered_OnWeb(float percentage);
void Update_LineStatus_OnWeb(int status);

int IsClientConnected(void);

#endif
