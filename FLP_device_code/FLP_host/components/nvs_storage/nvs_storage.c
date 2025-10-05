/*
 * nvs_storage.c
 *
 *  Created on: 2025. 10. 5.
 *      Author: user
 */


#include "nvs_storage.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <string.h>
#include "common/common_types.h"

static const char* TAG = "NVS_STORAGE";

static const char* INFO_NAMESPACE = "basic_info";
static const char* DEVICE_NAMESPACE = "device_info";

static const char* KEY_WIFI_SSID = "Wifi_SSID";
static const char* KEY_WIFI_PASS = "Wifi_pass";
static const char* KEY_DEV_SSID = "Device_SSID";
static const char* KEY_DEV_PASS = "Device_pass";
static const char* KEY_DEV_LIST = "device_struct";

esp_err_t nvs_storage_init(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS 파티션에 문제가 있어 새로 포맷합니다.");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS 초기화 성공");
    return ret;
}

esp_err_t nvs_storage_save_basic_info(const BasicInfoConfig* config) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(INFO_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;

    nvs_set_str(handle, KEY_WIFI_SSID, config->wifi_ssid);
    nvs_set_str(handle, KEY_WIFI_PASS, config->wifi_password);
    nvs_set_str(handle, KEY_DEV_SSID, config->device_ssid);
    nvs_set_str(handle, KEY_DEV_PASS, config->device_password);

    err = nvs_commit(handle);
    nvs_close(handle);
    return err;
}

esp_err_t nvs_storage_load_basic_info(BasicInfoConfig* config) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(INFO_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) return err;

    size_t len;
    len = sizeof(config->wifi_ssid);
    nvs_get_str(handle, KEY_WIFI_SSID, config->wifi_ssid, &len);
    len = sizeof(config->wifi_password);
    nvs_get_str(handle, KEY_WIFI_PASS, config->wifi_password, &len);
    len = sizeof(config->device_ssid);
    nvs_get_str(handle, KEY_DEV_SSID, config->device_ssid, &len);
    len = sizeof(config->device_password);
    nvs_get_str(handle, KEY_DEV_PASS, config->device_password, &len);

    nvs_close(handle);
    return ESP_OK;
}

esp_err_t nvs_storage_save_device_list(const int* device_list, size_t count) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(DEVICE_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;

    err = nvs_set_blob(handle, KEY_DEV_LIST, device_list, sizeof(int) * count);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }

    nvs_close(handle);
    return err;
}

esp_err_t nvs_storage_load_device_list(int* device_list, size_t* count, size_t max_size) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(DEVICE_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        *count = 0;
        return err;
    }

    size_t required_size = max_size * sizeof(int);
    err = nvs_get_blob(handle, KEY_DEV_LIST, device_list, &required_size);
    if (err == ESP_OK) {
        *count = required_size / sizeof(int);
    } else {
        *count = 0;
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGI(TAG, "저장된 장치 목록이 없습니다.");
        }
    }
    
    nvs_close(handle);
    return err;
}