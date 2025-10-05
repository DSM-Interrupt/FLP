/*
 * websocket_client.c
 *
 *  Created on: 2025. 10. 5.
 *      Author: user
 */


#include "websocket_client.h"
#include "esp_websocket_client.h"
#include "esp_log.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "wifi_manager.h"
#include "device_manager.h"

static const char* TAG = "WEBSOCKET_CLIENT";
#define HOST_IP_ADDR "wss://flp24.com:443"

static esp_websocket_client_handle_t client = NULL;

static void websocket_event_handler(void *handler_args, esp_event_base_t base,
                                    int32_t event_id, void *event_data) {
    esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;
    switch (event_id) {
        case WEBSOCKET_EVENT_CONNECTED:
            ESP_LOGI(TAG, "WebSocket 서버에 연결되었습니다.");
            break;
        case WEBSOCKET_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "WebSocket 연결이 끊겼습니다.");
            break;
        case WEBSOCKET_EVENT_DATA:
            ESP_LOGI(TAG, "데이터 수신. 길이: %d", data->data_len);
            if (data->data_len > 0) {
                 ESP_LOGI(TAG, "수신 데이터: %.*s", data->data_len, (char *)data->data_ptr);
                cJSON *root = cJSON_ParseWithLength((const char*)data->data_ptr, data->data_len);
                if (root) {
                    cJSON *danger_array = cJSON_GetObjectItem(root, "danger");
                    if (cJSON_IsArray(danger_array)) {
                        cJSON *item;
                        cJSON_ArrayForEach(item, danger_array) {
                            cJSON *id_json = cJSON_GetObjectItem(item, "id");
                            cJSON *distance_json = cJSON_GetObjectItem(item, "distance");
                            if (cJSON_IsNumber(id_json) && cJSON_IsNumber(distance_json)) {
                                TerminalDevice dev;
                                if(device_manager_get_device_by_id(id_json->valueint, &dev)) {
                                     device_manager_update_device_info(id_json->valueint, dev.Latit, dev.Longit, distance_json->valueint);
                                }
                            }
                        }
                    }
                    cJSON_Delete(root);
                }
            }
            break;
        case WEBSOCKET_EVENT_ERROR:
            ESP_LOGE(TAG, "WebSocket 오류 발생");
            break;
    }
}

static void websocket_connection_task(void *pvParameters) {
    while (1) {
        xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

        if (!websocket_client_is_connected()) {
            ESP_LOGI(TAG, "WebSocket 서버에 연결을 시도합니다...");
            esp_err_t err = esp_websocket_client_start(client);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "WebSocket 클라이언트 시작 실패: %s", esp_err_to_name(err));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// 공개 함수 구현
esp_err_t websocket_client_start(void) {
    if (client != NULL) {
        return ESP_OK; // 이미 초기화됨
    }
    const esp_websocket_client_config_t websocket_cfg = {
        .uri = HOST_IP_ADDR,
        .network_timeout_ms = 10000,
    };

    client = esp_websocket_client_init(&websocket_cfg);
    esp_websocket_register_events(client, WEBSOCKET_EVENT_ANY, websocket_event_handler, (void *)client);
    
    xTaskCreate(websocket_connection_task, "ws_conn_task", 4096, NULL, 5, NULL);

    return ESP_OK;
}

bool websocket_client_is_connected(void) {
    if (client == NULL) {
        return false;
    }
    return esp_websocket_client_is_connected(client);
}

esp_err_t websocket_client_send_text(const char* data) {
    if (!websocket_client_is_connected()) {
        ESP_LOGE(TAG, "WebSocket이 연결되지 않아 데이터를 보낼 수 없습니다.");
        return ESP_FAIL;
    }
    int len = strlen(data);
    int ret = esp_websocket_client_send_text(client, data, len, portMAX_DELAY);
    if (ret < 0) {
        ESP_LOGE(TAG, "WebSocket 데이터 전송 실패, 에러 코드: %d", ret);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "%d 바이트 데이터 전송 성공", len);
    return ESP_OK;
}