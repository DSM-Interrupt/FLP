/*
 * device_manager.c
 *
 *  Created on: 2025. 10. 5.
 *      Author: user
 */

#include "device_manager.h"
#include "nvs_storage.h"
#include "esp_log.h"
#include <string.h>

static const char* TAG = "DEVICE_MANAGER";

static TerminalDevice device_info_list[MAX_DEVICE_CNT];
static size_t device_count = 0;

void device_manager_init(void) {
    int temp_id_list[MAX_DEVICE_CNT];
    size_t loaded_count = 0;
    
    nvs_storage_load_device_list(temp_id_list, &loaded_count, MAX_DEVICE_CNT);

    device_count = loaded_count;
    for (int i = 0; i < device_count; i++) {
        device_info_list[i].Code = temp_id_list[i];
        device_info_list[i].Latit = 0.0;
        device_info_list[i].Longit = 0.0;
        device_info_list[i].Danger = 0;
    }
    ESP_LOGI(TAG, "%d개의 장치를 NVS에서 로드했습니다.", device_count);
}

bool device_manager_add_device(int device_id) {
    if (device_count >= MAX_DEVICE_CNT) {
        ESP_LOGE(TAG, "장치 목록이 가득 찼습니다. (최대: %d)", MAX_DEVICE_CNT);
        return false;
    }

    for (int i = 0; i < device_count; i++) {
        if (device_info_list[i].Code == device_id) {
            ESP_LOGW(TAG, "이미 등록된 장치 ID입니다: %d", device_id);
            return false;
        }
    }

    device_info_list[device_count].Code = device_id;
    device_info_list[device_count].Latit = 0.0;
    device_info_list[device_count].Longit = 0.0;
    device_info_list[device_count].Danger = 0;
    device_count++;

    int temp_id_list[MAX_DEVICE_CNT];
    for (int i = 0; i < device_count; i++) {
        temp_id_list[i] = device_info_list[i].Code;
    }

    if (nvs_storage_save_device_list(temp_id_list, device_count) == ESP_OK) {
        ESP_LOGI(TAG, "새로운 장치 ID %d를 추가하고 NVS에 저장했습니다. (현재 %d개)", device_id, device_count);
        return true;
    } else {
        ESP_LOGE(TAG, "NVS에 장치 목록 저장 실패");
        device_count--; 
        return false;
    }
}

size_t device_manager_get_device_count(void) {
    return device_count;
}

bool device_manager_get_device_by_index(size_t index, TerminalDevice *out_device) {
    if (index >= device_count || out_device == NULL) {
        return false;
    }
    memcpy(out_device, &device_info_list[index], sizeof(TerminalDevice));
    return true;
}

bool device_manager_update_device_info(int device_id, double lat, double lon, uint8_t danger) {
    for (int i = 0; i < device_count; i++) {
        if (device_info_list[i].Code == device_id) {
            device_info_list[i].Latit = lat;
            device_info_list[i].Longit = lon;
            device_info_list[i].Danger = danger;
            return true;
        }
    }
    ESP_LOGW(TAG, "업데이트하려는 장치 ID를 찾을 수 없습니다: %d", device_id);
    return false;
}

bool device_manager_get_device_by_id(int device_id, TerminalDevice *out_device) {
    if (out_device == NULL) {
        return false;
    }

    for (int i = 0; i < device_count; i++) {
        if (device_info_list[i].Code == device_id) {
            // ID가 일치하는 장치를 찾으면 정보를 복사하고 true를 반환합니다.
            memcpy(out_device, &device_info_list[i], sizeof(TerminalDevice));
            return true;
        }
    }

    // 루프가 끝날 때까지 장치를 찾지 못하면 false를 반환합니다.
    return false;
}