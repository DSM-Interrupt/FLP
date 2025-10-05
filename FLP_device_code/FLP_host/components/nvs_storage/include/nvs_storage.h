/*
 * nvs_storage.h
 *
 *  Created on: 2025. 10. 5.
 *      Author: user
 */

#ifndef NVS_STORAGE_H
#define NVS_STORAGE_H

#include "esp_err.h"
#include <stddef.h>

typedef struct {
    char wifi_ssid[32];
    char wifi_password[64];
    char device_ssid[32];
    char device_password[64];
} BasicInfoConfig;

/**
 * @brief NVS를 초기화하는 메서드.
 *
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t nvs_storage_init(void);

/**
 * @brief Wi-Fi 및 장치 AP 설정을 NVS에 저장.
 *
 * @param config 저장할 설정값이 담긴 구조체 포인터
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t nvs_storage_save_basic_info(const BasicInfoConfig* config);

/**
 * @brief Wi-Fi 및 장치 AP 설정을 NVS에서 불러오는 메서드.
 *
 * @param config 불러온 설정값을 저장할 구조체 포인터
 * @return esp_err_t ESP_OK on success, ESP_ERR_NVS_NOT_FOUND if not found.
 */
esp_err_t nvs_storage_load_basic_info(BasicInfoConfig* config);

/**
 * @brief 등록된 장치 ID 목록을 NVS에 저장.
 *
 * @param device_list 저장할 장치 ID 배열
 * @param count 저장할 장치의 수
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t nvs_storage_save_device_list(const int* device_list, size_t count);

/**
 * @brief 등록된 장치 ID 목록을 NVS에서 불러오는 메서드.
 *
 * @param device_list 불러온 장치 ID를 저장할 배열
 * @param count 불러온 장치의 수를 저장할 포인터 (입력 시 max_size, 출력 시 실제 개수)
 * @param max_size device_list 배열의 최대 크기
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t nvs_storage_load_device_list(int* device_list, size_t* count, size_t max_size);

#endif // NVS_STORAGE_H