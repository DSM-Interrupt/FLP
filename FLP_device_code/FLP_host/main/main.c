#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "cJSON.h"
#include <string.h>
#include <stdlib.h>

#include "common/common_types.h"
#include "nvs_storage.h"
#include "wifi_manager.h"
#include "lora_driver.h"
#include "gps_reader.h"
#include "device_manager.h"
#include "web_server.h"
#include "websocket_client.h"

static const char* TAG = "APP_MAIN";
static const int my_id_int = 1726384219;
static const char *my_id_char = "1726384219";

#define CTRL      GPIO_NUM_17
#define FIND_HOST GPIO_NUM_25

static bool split_lora_response(char* input, TerminalDevice* info) {
    char* token;
    token = strtok(input, ",");
    if (token == NULL || info->Code != atoi(token)) return false;
    
    token = strtok(NULL, ",");
    if (token == NULL) return false;
    info->Latit = atof(token);
    
    token = strtok(NULL, ",");
    if (token == NULL) return false;
    info->Longit = atof(token);
    
    token = strtok(NULL, ",");
    if (token == NULL || my_id_int != atoi(token)) return false;

    return true;
}

static char* make_json_payload(void) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "hostId", my_id_char);

    cJSON *members_array = cJSON_CreateArray();
    size_t count = device_manager_get_device_count();

    for (int i = 0; i < count; i++) {
        TerminalDevice dev;
        if (device_manager_get_device_by_index(i, &dev)) {
            cJSON *member = cJSON_CreateObject();
            cJSON_AddNumberToObject(member, "memberId", dev.Code);
            cJSON_AddNumberToObject(member, "lat", dev.Latit);
            cJSON_AddNumberToObject(member, "lon", dev.Longit);
            cJSON_AddItemToArray(members_array, member);
        }
    }
    cJSON_AddItemToObject(root, "members", members_array);

    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return json_str;
}

void lora_host_task(void *pvParameters) {
    const TickType_t timeout_ticks = pdMS_TO_TICKS(5000);

    while (1) {
        size_t device_count = device_manager_get_device_count();
        ESP_LOGI(TAG, "LoRa 태스크 시작, %d개의 장치를 순회합니다.", device_count);

        for (int i = 0; i < device_count; i++) {
            TerminalDevice current_device;
            if (!device_manager_get_device_by_index(i, &current_device)) continue;
            
            char gps_request_msg[64];
            snprintf(gps_request_msg, sizeof(gps_request_msg), "%u,%u,%u,getgps", current_device.Code, my_id_int, current_device.Danger);
            lora_send_packet(gps_request_msg);
            ESP_LOGI(TAG, "Sent to %d: %s", current_device.Code, gps_request_msg);

            TickType_t start_time = xTaskGetTickCount();
            bool received = false;
            while ((xTaskGetTickCount() - start_time) < timeout_ticks) {
                char buf[LORA_MAX_PACKET_SIZE] = {0};
                if (lora_receive_packet(buf) > 0) {
                    if (strstr(buf, my_id_char) != NULL) {
                        if (split_lora_response(buf, &current_device)) {
                            device_manager_update_device_info(current_device.Code, current_device.Latit, current_device.Longit, current_device.Danger);
                            ESP_LOGI(TAG, "Received from %d: Lat=%f, Lon=%f", current_device.Code, current_device.Latit, current_device.Longit);
                            received = true;
                            break;
                        }
                    }
                }
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            if (!received) {
                ESP_LOGW(TAG, "Device %d로부터 응답 시간 초과", current_device.Code);
            }
        }

        if (websocket_client_is_connected() && device_count > 0) {
            char* json_payload = make_json_payload();
            if (json_payload) {
                websocket_client_send_text(json_payload);
                free(json_payload);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


void app_main(void) {
    ESP_LOGI(TAG, "===== FLP Host 시스템 시작 =====");

    ESP_ERROR_CHECK(nvs_storage_init());
    ESP_ERROR_CHECK(wifi_manager_init_apsta());
    
    device_manager_init();
    ESP_ERROR_CHECK(lora_init());
    ESP_ERROR_CHECK(gps_reader_init());
    ESP_ERROR_CHECK(web_server_start());
    ESP_ERROR_CHECK(websocket_client_start());

    xTaskCreate(lora_host_task, "lora_host_task", 4096, NULL, 5, NULL);
    
    gpio_set_direction(CTRL, GPIO_MODE_INPUT);
    gpio_set_pull_mode(CTRL, GPIO_PULLDOWN_ONLY);
    gpio_set_direction(FIND_HOST, GPIO_MODE_INPUT);
    gpio_set_pull_mode(FIND_HOST, GPIO_PULLDOWN_ONLY);

    ESP_LOGI(TAG, "초기화 완료. 메인 루프 진입.");
    while (1) {
		if (gpio_get_level(CTRL) == 1 && gpio_get_level(FIND_HOST) == 1) {
			ESP_LOGI(TAG, "장치 등록 모드 진입 (10초)");
			TickType_t start_time = xTaskGetTickCount();
			while ((xTaskGetTickCount() - start_time) < pdMS_TO_TICKS(10000)) {
				char buf[LORA_MAX_PACKET_SIZE] = {0};
				if (lora_receive_packet(buf) > 0 && strstr(buf, "FINDHOST") != NULL) {
					int add_id = 0;
					char *token = strtok(buf, ",");
                    if (token) token = strtok(NULL, ",");
					if (token) add_id = atoi(token);

					if (add_id > 0) {
						ESP_LOGI(TAG, "장치 등록 요청 수신 ID: %d", add_id);
						if (device_manager_add_device(add_id)) {
							ESP_LOGI(TAG, "장치 %d 추가 성공", add_id);
						} else {
							ESP_LOGW(TAG, "장치 %d 추가 실패", add_id);
						}
					}
					break; 
				}
				vTaskDelay(pdMS_TO_TICKS(10));
			}
			ESP_LOGI(TAG, "장치 등록 모드 종료.");
		}
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}