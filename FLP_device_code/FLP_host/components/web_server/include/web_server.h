/*
 * web_server.h
 *
 *  Created on: 2025. 10. 5.
 *      Author: user
 */

#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include "esp_err.h"

/**
 * @brief 캡티브 포털 웹 서버와 DNS 서버를 시작.
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t web_server_start(void);

/**
 * @brief 실행 중인 웹 서버를 중지.
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t web_server_stop(void);

#endif // WEB_SERVER_H
