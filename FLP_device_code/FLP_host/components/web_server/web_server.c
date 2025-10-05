/*
 * web_server.c
 *
 *  Created on: 2025. 10. 5.
 *      Author: user
 */


#include "web_server.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "lwip/sockets.h"
#include <string.h>

#include "wifi_manager.h"
#include "device_manager.h"
#include "common/common_types.h"
#include "websocket_client.h"

#define DNS_PORT 53
#define DNS_MAX_LEN 256

static const char* TAG = "WEB_SERVER";
static httpd_handle_t server = NULL;

static const char html_start[] =
    "<!DOCTYPE html>"
    "<html>"
    "<head>"
    "<meta name='viewport' content='width=device-width, initial-scale=1'>"
    "<title>FLP Host-Device Config</title>"
    "<style>"
    "body { font-family: Arial, sans-serif; background-color: #f4f4f4; margin: 20px; }"
    "h2 { color: #333; }"
    "form { background-color: #fff; padding: 20px; border-radius: 5px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }"
    "table { width: 100%%; }"
    "td { padding: 8px; }"
    "td:first-child { font-weight: bold; width: 40%%; }"
    "input[type='text'], input[type='password'] { width: 95%%; padding: 8px; border: 1px solid #ddd; border-radius: 4px; }"
    "input[type='submit'] { background-color: #4CAF50; color: white; padding: 10px 15px; border: none; border-radius: 4px; cursor: pointer; }"
    "input[type='submit']:hover { background-color: #45a049; }"
    ".status { background-color: #eef; padding: 20px; border-radius: 5px; margin-top: 20px; }"
    "</style>"
    "</head>"
    "<body>"
    "<h2> Wi-Fi & Device Settings </h2>"
    "<form action='/submit' method='post'>"
    "<table>"
    "<tr><td>Wi-Fi SSID:</td><td><input type='text' name='SSID' value='%s'></td></tr>"
    "<tr><td>Wi-Fi Password:</td><td><input type='password' name='Wifipass' value='%s'></td></tr>"
    "<tr><td>Device AP SSID:</td><td><input type='text' name='Device_SSID' value='%s'></td></tr>"
    "<tr><td>Device AP Password:</td><td><input type='password' name='Device_Wifipass' value='%s'></td></tr>"
    "</table>"
    "<br><input type='submit' value='Save & Restart'>"
    "</form>"
    "<div class='status'>"
    "<h2>Device Status</h2>"
    "<b>Connected Clients (AP):</b> %d<br>"
    "<b>Server Connection:</b> %s<br>"
    "<b>Registered Device IDs:</b> %s<br>"
    "</div>"
    "</body>"
    "</html>";

static void dns_server_task(void *pvParameters) {
    uint8_t buffer[DNS_MAX_LEN];
    struct sockaddr_in client_addr;
    socklen_t client_addr_len = sizeof(client_addr);
    struct sockaddr_in server_addr;

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "DNS 소켓 생성 실패");
        vTaskDelete(NULL);
    }

    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(DNS_PORT);

    if (bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "DNS Bind 실패");
        close(sock);
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "DNS 서버가 시작되었습니다.");

    while (1) {
        int len = recvfrom(sock, buffer, sizeof(buffer), 0, (struct sockaddr *)&client_addr, &client_addr_len);
        if (len > 0) {
            buffer[2] |= 0x80;
            buffer[2] |= 0x04;
            buffer[3] |= 0x80;

            esp_netif_ip_info_t ip_info;
            esp_netif_t *ap_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
            esp_netif_get_ip_info(ap_netif, &ip_info);

            uint8_t *dns_response = buffer + len;
            *dns_response++ = 0xC0; *dns_response++ = 0x0C;
            *dns_response++ = 0x00; *dns_response++ = 0x01;
            *dns_response++ = 0x00; *dns_response++ = 0x01;
            *dns_response++ = 0x00; *dns_response++ = 0x00; *dns_response++ = 0x00; *dns_response++ = 0x78; // TTL
            *dns_response++ = 0x00; *dns_response++ = 0x04;

            uint32_t ip_addr = ip_info.ip.addr;
            *dns_response++ = (uint8_t)(ip_addr >> 0);
            *dns_response++ = (uint8_t)(ip_addr >> 8);
            *dns_response++ = (uint8_t)(ip_addr >> 16);
            *dns_response++ = (uint8_t)(ip_addr >> 24);

            sendto(sock, buffer, len + 16, 0, (struct sockaddr *)&client_addr, client_addr_len);
        }
    }
    close(sock);
    vTaskDelete(NULL);
}


static esp_err_t submit_post_handler(httpd_req_t *req) {
    char buf[256];
    
    BasicInfoConfig new_config;
    wifi_manager_get_current_config(&new_config);

    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        char param[64];
        if (httpd_query_key_value(buf, "SSID", param, sizeof(param)) == ESP_OK) {
            strncpy(new_config.wifi_ssid, param, sizeof(new_config.wifi_ssid) - 1);
        }
        if (httpd_query_key_value(buf, "Wifipass", param, sizeof(param)) == ESP_OK) {
            strncpy(new_config.wifi_password, param, sizeof(new_config.wifi_password) - 1);
        }
        if (httpd_query_key_value(buf, "Device_SSID", param, sizeof(param)) == ESP_OK) {
            strncpy(new_config.device_ssid, param, sizeof(new_config.device_ssid) - 1);
        }
        if (httpd_query_key_value(buf, "Device_Wifipass", param, sizeof(param)) == ESP_OK) {
            strncpy(new_config.device_password, param, sizeof(new_config.device_password) - 1);
        }
    }
    
    wifi_manager_update_config(&new_config);

    const char* resp_str = "<h2>Settings Updated!</h2>";
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t captive_portal_get_handler(httpd_req_t *req) {
    char html_buffer[4096];
    
    BasicInfoConfig current_config;
    wifi_manager_get_current_config(&current_config);
    
    wifi_sta_list_t sta_list;
    esp_wifi_ap_get_sta_list(&sta_list);
    
    const char *server_status = websocket_client_is_connected() ? "Connected" : "Disconnected";
    
    char device_status_str[MAX_DEVICE_CNT * 12 + 20] = "No devices registered";
    size_t device_count = device_manager_get_device_count();
    if (device_count > 0) {
        char temp_str[256] = {0};
        for (int i = 0; i < device_count; i++) {
            TerminalDevice dev;
            if (device_manager_get_device_by_index(i, &dev)) {
                char id_str[12];
                sprintf(id_str, "%d, ", dev.Code);
                strcat(temp_str, id_str);
            }
        }
        if (strlen(temp_str) > 2) {
            temp_str[strlen(temp_str) - 2] = '\0';
        }
		strcpy(device_status_str, temp_str);
    }
    
    snprintf(html_buffer, sizeof(html_buffer), html_start,
        current_config.wifi_ssid, "********",
        current_config.device_ssid, "********",
        sta_list.num, server_status, device_status_str
    );

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html_buffer, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static const httpd_uri_t submit_uri = {
    .uri       = "/submit",
    .method    = HTTP_POST,
    .handler   = submit_post_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t captive_portal_uri = {
    .uri       = "/*", 
    .method    = HTTP_GET,
    .handler   = captive_portal_get_handler,
    .user_ctx  = NULL
};

esp_err_t web_server_start(void) {
    if (server != NULL) {
        return ESP_OK;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;

    ESP_LOGI(TAG, "웹 서버 시작 중...");
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &submit_uri);
        httpd_register_uri_handler(server, &captive_portal_uri);
        xTaskCreate(dns_server_task, "dns_server", 4096, NULL, 5, NULL);
        return ESP_OK;
    }
    
    ESP_LOGE(TAG, "웹 서버 시작 실패");
    return ESP_FAIL;
}

esp_err_t web_server_stop(void) {
    if (server != NULL) {
        httpd_stop(server);
        server = NULL;
    }
    return ESP_OK;
}