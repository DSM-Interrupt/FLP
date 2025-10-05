/*
 * gps_reader.h
 *
 *  Created on: 2025. 10. 5.
 *      Author: user
 */

#ifndef GPS_READER_H
#define GPS_READER_H

#include "esp_err.h"
#include "hal/gpio_types.h"
#include <stdbool.h>

#define GPS_TX       GPIO_NUM_32
#define GPS_RX       GPIO_NUM_33
#define UART_GPS     UART_NUM_1
#define UART_BUF_SIZE 1024

/**
 * @brief GPS 리더를 초기화하고, 좌표를 읽는 내부 태스크 시작하기 위한 함수
 *
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t gps_reader_init(void);

/**
 * @brief 가장 최근에 수신된 GPS 좌표를 가져오는 메서드
 *
 * @param latitude 위도를 저장할 포인터
 * @param longitude 경도를 저장할 포인터
 * @return true 좌표가 유효한 경우
 * @return false 아직 유효한 좌표를 수신하지 못한 경우
 */
bool gps_reader_get_coordinates(double *latitude, double *longitude);

#endif // GPS_READER_H
