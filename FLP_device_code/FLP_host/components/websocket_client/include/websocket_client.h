/*
 * websocket_client.h
 *
 *  Created on: 2025. 10. 5.
 *      Author: user
 */

#ifndef WEBSOCKET_CLIENT_H
#define WEBSOCKET_CLIENT_H

#include "esp_err.h"
#include <stdbool.h>

/**
 * @brief WebSocket 클라이언트를 초기화하고 서버 연결 태스크를 시작.
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t websocket_client_start(void);

/**
 * @brief WebSocket이 서버에 연결되어 있는지 확인.
 *
 * @return true 연결됨
 * @return false 연결되지 않음
 */
bool websocket_client_is_connected(void);

/**
 * @brief WebSocket을 통해 서버로 텍스트(JSON) 데이터를 전송.
 *
 * @param data 전송할 문자열 데이터
 * @return esp_err_t ESP_OK on success
 */
esp_err_t websocket_client_send_text(const char* data);

#endif // WEBSOCKET_CLIENT_H