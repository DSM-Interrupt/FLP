/*
 * gps_reader.c
 *
 *  Created on: 2025. 10. 5.
 *      Author: user
 */




#include "gps_reader.h"
#include <string.h>
#include <stdlib.h>
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "GPS";
static double current_latitude = 0.0;
static double current_longitude = 0.0;
static bool has_valid_coordinates = false;

static double convert_to_decimal_degrees(const char* nmea_coord, char direction) {
    double raw = atof(nmea_coord);
    int degrees = (int)(raw / 100);
    double minutes = raw - (degrees * 100);
    double decimal_degrees = degrees + (minutes / 60.0);

    if (direction == 'S' || direction == 'W') {
        decimal_degrees *= -1.0;
    }
    return decimal_degrees;
}

static void parse_nmea_sentence(char* sentence) {
    if (strstr(sentence, "$GPGGA") || strstr(sentence, "$GPRMC")) {
        char *token;
        int field_index = 0;
        char lat[16] = {0}, lon[16] = {0};
        char lat_dir = 0, lon_dir = 0;

        token = strtok(sentence, ",");
        while (token != NULL) {
            field_index++;
            if (field_index == 3) strncpy(lat, token, sizeof(lat) - 1);
            if (field_index == 4) lat_dir = token[0];
            if (field_index == 5) strncpy(lon, token, sizeof(lon) - 1);
            if (field_index == 6) lon_dir = token[0];
            token = strtok(NULL, ",");
        }

        if (strlen(lat) > 0 && strlen(lon) > 0 && lat_dir != 0 && lon_dir != 0) {
            current_latitude = convert_to_decimal_degrees(lat, lat_dir);
            current_longitude = convert_to_decimal_degrees(lon, lon_dir);
            has_valid_coordinates = true;
            ESP_LOGI(TAG, "위도: %lf, 경도: %lf", current_latitude, current_longitude);
        }
    }
}

static void gps_task(void *arg) {
    uint8_t* data = (uint8_t*) malloc(UART_BUF_SIZE);
    while (1) {
        int len = uart_read_bytes(UART_GPS, data, UART_BUF_SIZE - 1, pdMS_TO_TICKS(1000));
        if (len > 0) {
            data[len] = '\0';
            char* line = strtok((char*)data, "\r\n");
            while (line != NULL) {
                char temp_line[256];
                strncpy(temp_line, line, sizeof(temp_line)-1);
                temp_line[sizeof(temp_line)-1] = '\0';
                parse_nmea_sentence(temp_line);
                line = strtok(NULL, "\r\n");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    free(data);
}

// 공개 함수 구현
esp_err_t gps_reader_init(void) {
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    
    ESP_ERROR_CHECK(uart_driver_install(UART_GPS, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_GPS, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_GPS, GPS_TX, GPS_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    xTaskCreate(gps_task, "gps_task", 4096, NULL, 10, NULL);
    ESP_LOGI(TAG, "GPS reader 초기화 및 태스크 생성 완료");
    
    return ESP_OK;
}

bool gps_reader_get_coordinates(double *latitude, double *longitude) {
    if (has_valid_coordinates && latitude != NULL && longitude != NULL) {
        *latitude = current_latitude;
        *longitude = current_longitude;
        return true;
    }
    return false;
}