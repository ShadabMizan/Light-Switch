#include "WebServer.h"
#include "Externals.h"

#include "esp_https_server.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_spiffs.h"

/* Private Resources */
static const char *TAG = "WEBSERVER";

static httpd_req_t *sse_client = NULL;

static esp_err_t serve_file(httpd_req_t *req);
static esp_err_t api_toggle_relay(httpd_req_t *req);
static esp_err_t api_set_power(httpd_req_t *req);
static esp_err_t api_get_power(httpd_req_t *req);
static esp_err_t api_get_line_status(httpd_req_t *req);

static void start_webserver(void);

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);

/* Public Function */
void BackEnd_Init(const char *ssid, const char *password) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {0};
    strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char*)wifi_config.sta.password, password, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Mount SPIFFS
    esp_vfs_spiffs_conf_t spiffs_conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 8,
        .format_if_mount_failed = true
    };
    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&spiffs_conf));
}

/* WiFi Event Handler */
void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_err_t status = esp_wifi_connect();
        if (status != ESP_OK) {
            ESP_LOGW(TAG, "Failed to connect to WiFi: %s", esp_err_to_name(status));
            Blink_LED(RED_LED, 2);
        }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_err_t status = esp_wifi_connect();
        if (status != ESP_OK) {
            ESP_LOGW(TAG, "Failed to connect to WiFi: %s", esp_err_to_name(status));
            Blink_LED(RED_LED, 2);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        Blink_LED(GREEN_LED, 2);
        start_webserver();
    }
}

/* Webserver Setup */
void start_webserver(void) {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    if (httpd_start(&server, &config) == ESP_OK) {
        // Static files
        httpd_uri_t static_files = {
            .uri = "/*",
            .method = HTTP_GET,
            .handler = serve_file
        };
        httpd_register_uri_handler(server, &static_files);

        // API routes
        httpd_uri_t toggle_relay = {
            .uri = "/api/toggle",
            .method = HTTP_POST,
            .handler = api_toggle_relay
        };
        httpd_register_uri_handler(server, &toggle_relay);

        httpd_uri_t set_power = {
            .uri = "/api/set_power",
            .method = HTTP_POST,
            .handler = api_set_power
        };
        httpd_register_uri_handler(server, &set_power);

        httpd_uri_t get_power = {
            .uri = "/api/get_power",
            .method = HTTP_GET,
            .handler = api_get_power
        };
        httpd_register_uri_handler(server, &get_power);

        httpd_uri_t get_line_status = {
            .uri = "/api/get_line_status",
            .method = HTTP_GET,
            .handler = api_get_line_status
        };
        httpd_register_uri_handler(server, &get_line_status);
    } else {
        ESP_LOGE(TAG, "HTTPD Start Failed");
        Blink_LED(RED_LED, 3);
    }
}

/* HTTP Handlers */
esp_err_t serve_file(httpd_req_t *req) {
    char filepath[128];
    const char *filename = (strlen(req->uri) == 1) ? "/index.html" : req->uri;
    snprintf(filepath, sizeof(filepath), "/spiffs%.120s", filename);

    FILE *file = fopen(filepath, "r");
    if (!file) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    char buffer[512];
    size_t read_bytes;
    while ((read_bytes = fread(buffer, 1, sizeof(buffer), file)) > 0) {
        httpd_resp_send_chunk(req, buffer, read_bytes);
    }
    fclose(file);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

esp_err_t api_toggle_relay(httpd_req_t *req) {
    Toggle_Relay();
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);

    Blink_LED(GREEN_LED, 1);
    return ESP_OK;
}

esp_err_t api_set_power(httpd_req_t *req) {
    char buf[16];
    int len = httpd_req_recv(req, buf, sizeof(buf)-1);
    if (len > 0) {
        buf[len] = '\0';
        float percentage = atof(buf);
        Set_PowerDelivered(percentage);
    }
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t api_get_power(httpd_req_t *req) {
    char resp[32];
    snprintf(resp, sizeof(resp), "%.3f", Get_PowerDelivered());
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t api_get_line_status(httpd_req_t *req) {
    char resp[8];
    snprintf(resp, sizeof(resp), "%d", Get_LineStatus());
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/* Server Sent Events */

void sse_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/event-stream");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    sse_client = req;

    while (1) {
        // Keep connection open
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void UpdatePowerDeliveredOnWeb(float percentage) {
    if (!sse_client) return;

    char buf[64];
    int len = snprintf(buf, sizeof(buf), "data: %.3f\n\n", percentage);
    httpd_resp_send_chunk(sse_client, buf, len);
}
