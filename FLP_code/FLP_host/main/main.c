#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "soc/gpio_num.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "freertos/event_groups.h"
#include "esp_netif_ip_addr.h"
#include "driver/spi_master.h"

#define HOST_IP_ADDR "192.168.0.100"
#define PORT 3333

#define WIFI_SSID "Your_SSID"
#define WIFI_PASS "Your_PASSWORD"

#define WIFI_CONNECTED_BIT BIT0
#define PIN_NUM_MISO GPIO_NUM_19
#define PIN_NUM_MOSI GPIO_NUM_23
#define PIN_NUM_CLK  GPIO_NUM_18
#define PIN_NUM_CS   GPIO_NUM_5
#define PIN_NUM_RST  GPIO_NUM_27
#define PIN_NUM_IRQ  GPIO_NUM_26

static const char *LR_TAG = "LoRa";
static EventGroupHandle_t wifi_event_group;
static const char *TAG = "wifi";

spi_device_handle_t spi;

void lora_write_byte(uint8_t addr, uint8_t data) {
    uint8_t tx_data[2] = { addr | 0x80, data };
    spi_transaction_t t = {
        .length = 8 * 2,
        .tx_buffer = tx_data,
        .rx_buffer = NULL
    };
    spi_device_transmit(spi, &t);
}

uint8_t lora_read_byte(uint8_t addr) {
    uint8_t tx_data[2] = { addr & 0x7F, 0x00 };
    uint8_t rx_data[2];
    spi_transaction_t t = {
        .length = 8 * 2,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data
    };
    spi_device_transmit(spi, &t);
    return rx_data[1];
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    	ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
    	char ip_str[INET_ADDRSTRLEN];
    	esp_ip4addr_ntoa(&event->ip_info.ip, ip_str, INET_ADDRSTRLEN);
    	ESP_LOGI(TAG, "Got IP: %s", ip_str);
    	xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
	}
}

void wifi_init_sta(void) {
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT,
                        false, true, portMAX_DELAY);
}

void tcp_client_task(void *pvParameters) {
    char rx_buffer[128];
    char host_ip[] = HOST_IP_ADDR;
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;

    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(host_ip);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);

    int sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (sock < 0) {
        ESP_LOGE("TCP", "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE("TCP", "Socket unable to connect: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        int err = send(sock, "Hello Server", strlen("Hello Server"), 0);
        if (err < 0) {
            ESP_LOGE("TCP", "Error occurred during sending: errno %d", errno);
            break;
        }

        int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
        if (len < 0) {
            ESP_LOGE("TCP", "recv failed: errno %d", errno);
            break;
        } else {
            rx_buffer[len] = 0; // null-terminate
            ESP_LOGI("TCP", "Received: %s", rx_buffer);
        }
        
    }

    close(sock);
    vTaskDelete(NULL);
}

void app_main(void) {
	
	gpio_num_t pins[] = {GPIO_NUM_17, GPIO_NUM_25, GPIO_NUM_26, GPIO_NUM_27};
	for (int i = 0; i < sizeof(pins)/sizeof(pins[0]); i++) {
	    gpio_set_direction(pins[i], GPIO_MODE_INPUT);
	    gpio_set_pull_mode(pins[i], GPIO_FLOATING);
	}
	
	wifi_init_sta();
	
	xTaskCreate(tcp_client_task, "tcp_client", 4096, NULL, 5, NULL);
    while (1) {
        
    }
}