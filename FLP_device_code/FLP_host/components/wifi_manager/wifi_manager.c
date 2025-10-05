/*
 * wifi_manager.c
 *
 *  Created on: 2025. 10. 5.
 *      Author: user
 */

#include "wifi_manager.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include <string.h>

static const char *TAG = "WIFI_MANAGER";

EventGroupHandle_t wifi_event_group;

static BasicInfoConfig current_config;

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "Station 모드 시작, Wi-Fi 연결 시도 중...");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Wi-Fi 연결 끊김. 재연결 시도...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Wi-Fi 연결 성공! IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "클라이언트가 SoftAP에 연결되었습니다. MAC: %02x:%02x:%02x:%02x:%02x:%02x",
                 event->mac[0], event->mac[1], event->mac[2], event->mac[3], event->mac[4], event->mac[5]);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "클라이언트 연결이 끊겼습니다. MAC: %02x:%02x:%02x:%02x:%02x:%02x",
                 event->mac[0], event->mac[1], event->mac[2], event->mac[3], event->mac[4], event->mac[5]);
    }
}

esp_err_t wifi_manager_init_apsta(void) {
    wifi_event_group = xEventGroupCreate();

    esp_err_t ret = nvs_storage_load_basic_info(&current_config);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "저장된 Wi-Fi 설정이 없습니다. 기본값으로 시작합니다.");
        strcpy(current_config.wifi_ssid, "Your_SSID");
        strcpy(current_config.wifi_password, "Your_PASSWORD");
        strcpy(current_config.device_ssid, "FLP_HOST");
        strcpy(current_config.device_password, "Default");
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    
    wifi_config_t sta_config = {
        .sta = { .threshold.authmode = WIFI_AUTH_WPA2_PSK },
    };
    strncpy((char*)sta_config.sta.ssid, current_config.wifi_ssid, sizeof(sta_config.sta.ssid));
    strncpy((char*)sta_config.sta.password, current_config.wifi_password, sizeof(sta_config.sta.password));
    
    wifi_config_t ap_config = {
        .ap = {
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    strncpy((char*)ap_config.ap.ssid, current_config.device_ssid, sizeof(ap_config.ap.ssid));
    strncpy((char*)ap_config.ap.password, current_config.device_password, sizeof(ap_config.ap.password));
    if (strlen(current_config.device_password) == 0) {
        ap_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi AP+STA 초기화 완료. AP SSID: %s", current_config.device_ssid);

    return ESP_OK;
}

void wifi_manager_get_current_config(BasicInfoConfig* config) {
    if (config != NULL) {
        memcpy(config, &current_config, sizeof(BasicInfoConfig));
    }
}

esp_err_t wifi_manager_update_config(const BasicInfoConfig* config) {
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memcpy(&current_config, config, sizeof(BasicInfoConfig));

    nvs_storage_save_basic_info(&current_config);

    ESP_LOGI(TAG, "Wi-Fi 설정 변경... 재연결을 시작합니다.");
    ESP_ERROR_CHECK(esp_wifi_disconnect());

    wifi_config_t sta_config = {
        .sta = { .threshold.authmode = WIFI_AUTH_WPA2_PSK },
    };
    strncpy((char*)sta_config.sta.ssid, current_config.wifi_ssid, sizeof(sta_config.sta.ssid));
    strncpy((char*)sta_config.sta.password, current_config.wifi_password, sizeof(sta_config.sta.password));
    
    wifi_config_t ap_config = {
        .ap = {
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    strncpy((char*)ap_config.ap.ssid, current_config.device_ssid, sizeof(ap_config.ap.ssid));
    strncpy((char*)ap_config.ap.password, current_config.device_password, sizeof(ap_config.ap.password));
    if (strlen(current_config.device_password) == 0) {
        ap_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    
    return esp_wifi_connect();
}