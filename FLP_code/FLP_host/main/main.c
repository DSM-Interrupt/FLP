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

#define LORA_SPI_HOST HSPI_HOST
#define LORA_FREQUENCY 433000000

static const char *LR_TAG = "LoRa";
static EventGroupHandle_t wifi_event_group;
static const char *TAG = "wifi";

spi_device_handle_t lora_spi;

typedef struct terminaldevice{
	int code;
	double latit;
	double longit;
} TerminalDevice;

void lora_write_register(uint8_t reg, uint8_t val) {
    spi_transaction_t t = {
        .length = 8 * 2,
        .tx_buffer = (uint8_t[]){ reg | 0x80, val }
    };
    spi_device_transmit(lora_spi, &t);
}

uint8_t lora_read_register(uint8_t reg) {
    uint8_t out;
    uint8_t tx[2] = { reg & 0x7F, 0x00 };
    uint8_t rx[2];
    spi_transaction_t t = {
        .length = 8 * 2,
        .tx_buffer = tx,
        .rx_buffer = rx
    };
    spi_device_transmit(lora_spi, &t);
    out = rx[1];
    return out;
}

// 초기화
void lora_init() {
    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    // LoRa 모드로 설정
    lora_write_register(0x01, 0x80); // RegOpMode = LoRa + sleep
    lora_write_register(0x01, 0x81); // standby

    // 주파수 설정 (433MHz = 0x6C + 0x80 + 0x00)
    uint64_t frf = ((uint64_t)LORA_FREQUENCY << 19) / 32000000;
    lora_write_register(0x06, (frf >> 16) & 0xFF);
    lora_write_register(0x07, (frf >> 8) & 0xFF);
    lora_write_register(0x08, (frf >> 0) & 0xFF);

    // BW = 125kHz, CR = 4/5, Explicit Header
    lora_write_register(0x1D, 0x72); // RegModemConfig1
    lora_write_register(0x1E, 0xA4); // RegModemConfig2 (SF10)

    // preamble length
    lora_write_register(0x20, 0x00);
    lora_write_register(0x21, 0x08);

    // payload CRC
    lora_write_register(0x1F, 0x00); // disable CRC
}

// 패킷 송신
void lora_send_packet(const char* data) {
    size_t len = strlen(data);

    // standby 모드
    lora_write_register(0x01, 0x81);

    // FIFO 포인터 초기화
    lora_write_register(0x0E, 0x00); // FIFO TX base
    lora_write_register(0x0D, 0x00); // FIFO addr ptr

    for (int i = 0; i < len; i++) {
        lora_write_register(0x00, data[i]);
    }

    lora_write_register(0x22, len); // payload length
    lora_write_register(0x01, 0x83); // TX mode

    // 전송 완료 대기
    while ((lora_read_register(0x12) & 0x08) == 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // IRQ 플래그 클리어
    lora_write_register(0x12, 0xFF);
}
/**/
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
		
        
    }

    close(sock);
    vTaskDelete(NULL);
}



void app_main(void) {
	spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1*1000*1000,
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 7
    };
	
	
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