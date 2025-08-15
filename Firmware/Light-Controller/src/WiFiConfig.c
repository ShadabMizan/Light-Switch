#include "WiFiConfig.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_spiffs.h"
#include "esp_https_server.h"

static const char *TAG = "WIFI_CONFIG";
static wifi_config_data_t g_config;

static esp_err_t save_config(const wifi_config_data_t *config) {
    nvs_handle_t handle;
    ESP_ERROR_CHECK(nvs_open("wifi_cfg", NVS_READWRITE, &handle));
    ESP_ERROR_CHECK(nvs_set_blob(handle, "config", config, sizeof(wifi_config_data_t)));
    ESP_ERROR_CHECK(nvs_commit(handle));
    nvs_close(handle);
    return ESP_OK;
}

esp_err_t WiFiConfig_Load(wifi_config_data_t *config) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open("wifi_cfg", NVS_READONLY, &handle);
    if (err != ESP_OK) return err;
    size_t len = sizeof(wifi_config_data_t);
    err = nvs_get_blob(handle, "config", config, &len);
    nvs_close(handle);
    return err;
}

static esp_err_t serve_file(httpd_req_t *req) {
    char filepath[128];
    const char *filename = (strlen(req->uri) == 1) ? "/setup/index.html" : req->uri;
    snprintf(filepath, sizeof(filepath), "/spiffs%.120s", filename);

    FILE *file = fopen(filepath, "r");
    if (!file) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    char buffer[256];
    size_t read_bytes;
    while ((read_bytes = fread(buffer, 1, sizeof(buffer), file)) > 0) {
        httpd_resp_send_chunk(req, buffer, read_bytes);
    }
    fclose(file);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t save_handler(httpd_req_t *req) {
    char buf[256];
    int len = httpd_req_recv(req, buf, sizeof(buf)-1);
    if (len <= 0) return ESP_FAIL;
    buf[len] = '\0';

    // Very basic form parsing (URL encoded: ssid=...&password=...&device=...)
    sscanf(buf, "ssid=%31s&password=%63s&device=%31s",
           g_config.ssid, g_config.password, g_config.device_name);

    save_config(&g_config);
    httpd_resp_sendstr(req, "OK");

    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

static esp_err_t start_ap_server(void) {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t static_files = {
            .uri = "/*",
            .method = HTTP_GET,
            .handler = serve_file
        };
        httpd_uri_t save_uri = {
            .uri = "/save",
            .method = HTTP_POST,
            .handler = save_handler
        };
        httpd_register_uri_handler(server, &static_files);
        httpd_register_uri_handler(server, &save_uri);
        return ESP_OK;
    }
    return ESP_FAIL;
}

esp_err_t WiFiConfig_StartPortal(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t ap_config = {
        .ap = {
            .ssid = "ESP32_Setup",
            .ssid_len = 0,
            .channel = 1,
            .max_connection = 4,
            .authmode = WIFI_AUTH_OPEN
        }
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Mount SPIFFS containing setup HTML/CSS/JS
    esp_vfs_spiffs_conf_t spiffs_conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 4,
        .format_if_mount_failed = true
    };
    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&spiffs_conf));

    return start_ap_server();
}
