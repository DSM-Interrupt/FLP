/*
 * lora_driver.h
 *
 *  Created on: 2025. 10. 5.
 *      Author: user
 */

#ifndef LORA_DRIVER_H
#define LORA_DRIVER_H

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"

#define PIN_NUM_MISO GPIO_NUM_19
#define PIN_NUM_MOSI GPIO_NUM_23
#define PIN_NUM_CLK  GPIO_NUM_5
#define PIN_NUM_CS   GPIO_NUM_18
#define PIN_NUM_RST  GPIO_NUM_16
#define PIN_NUM_IRQ  GPIO_NUM_26

#define LORA_SPI_HOST HSPI_HOST
#define LORA_FREQUENCY 433000000
#define LORA_MAX_PACKET_SIZE 256

/**
 * @brief LoRa 모듈을 초기화
 *
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t lora_init(void);

/**
 * @brief LoRa 패킷을 전송
 *
 * @param data 보낼 데이터 문자열
 */
void lora_send_packet(const char* data);

/**
 * @brief LoRa 패킷을 수신
 *
 * @param buf 수신한 데이터를 저장할 버퍼
 * @return int 수신한 데이터의 길이. 수신한 데이터가 없으면 0.
 */
int lora_receive_packet(char *buf);

#endif // LORA_DRIVER_H
