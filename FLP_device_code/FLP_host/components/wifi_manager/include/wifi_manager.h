/*
 * wifi_manager.h
 *
 *  Created on: 2025. 10. 5.
 *      Author: user
 */

#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include "freertos/FreeRTOS.h"
#include "esp_err.h"
#include "freertos/event_groups.h"
#include "nvs_storage.h"

#define WIFI_CONNECTED_BIT BIT0

/**
 * @brief 다른 모듈에서 Wi-Fi 연결 상태를 확인할 수 있도록 외부에 공개하는 이벤트 그룹 핸들
 */
extern EventGroupHandle_t wifi_event_group;

/**
 * @brief Wi-Fi를 AP+STA 모드로 초기화하고 연결을 시작.
 * 내부적으로 NVS에서 저장된 Wi-Fi 설정을 호출
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t wifi_manager_init_apsta(void);

/**
 * @brief 현재 적용된 Wi-Fi 및 AP 설정을 호출.
 *
 * @param config 설정을 복사해 갈 구조체 포인터
 */
void wifi_manager_get_current_config(BasicInfoConfig* config);

/**
 * @brief 새로운 Wi-Fi 및 AP 설정을 적용하고 NVS에 저장.
 *
 * @param config 적용할 새로운 설정
 * @return esp_err_t ESP_OK on success
 */
esp_err_t wifi_manager_update_config(const BasicInfoConfig* config);

#endif // WIFI_MANAGER_H