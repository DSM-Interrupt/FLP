#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/_intsup.h>
#include <sys/_types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/unistd.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "hal/gpio_types.h"
#include "lwip/sockets.h"
#include "soc/gpio_num.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs.h"
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
#define PIN_NUM_CLK  GPIO_NUM_5
#define PIN_NUM_CS   GPIO_NUM_18
#define PIN_NUM_RST  GPIO_NUM_16
#define PIN_NUM_IRQ  GPIO_NUM_26
#define GPS_RX GPIO_NUM_33
#define GPS_TX GPIO_NUM_32

#define CTRL GPIO_NUM_17
#define FIND_HOST GPIO_NUM_25
#define Enter_AND_SPACE GPIO_NUM_27
#define UP GPIO_NUM_34
#define DOWN GPIO_NUM_35

#define LORA_SPI_HOST HSPI_HOST
#define LORA_FREQUENCY 433000000
#define LORA_MAX_PACKET_SIZE 256

#define MAX_DEVICE_CNT 50
#define FLASH_STORAGE "storage"

#define TAG_NVS "nvs_init"

static EventGroupHandle_t wifi_event_group;
static const char *TAG = "wifi";
const unsigned int MY_ID_INT = 3582194827;
const char* MY_ID_CHAR = "3582194827";

bool gps_data_all_receive = false;

spi_device_handle_t lora_spi;

typedef struct terminaldevice{
	unsigned int code;
	double latit;
	double longit;
} TerminalDevice;

unsigned int device_list[MAX_DEVICE_CNT];
TerminalDevice* device_info[MAX_DEVICE_CNT];
size_t device_count = 0;

unsigned int atou(const char* str) {
    int result = 0;

    while (isspace(*str)) {
        str++;
    }

    while (isdigit(*str)) {
        result = result * 10 + (*str - '0');
        str++;
    }

    return result;
}

char* make_json_payload(void) {
    char* json = malloc(4096);
    if (!json) return NULL;
    sprintf(json, "{\n\"hostId\": \"%s\",\n\"members\": [\n", MY_ID_CHAR);
    char entry[256];
    int first = 1;
    for (int i = 0; i < MAX_DEVICE_CNT; i++) {
        if (!device_info[i]) continue;
        if (!first) {
            strcat(json, ",\n");
        }
        first = 0;
        snprintf(entry, sizeof(entry),
            "{\n\"memberId\": \"%u\",\n\"lat\": %.6f,\n\"lon\": %.6f\n}",
            device_info[i]->code,
            device_info[i]->latit,
            device_info[i]->longit
        );
        strcat(json, entry);
    }
    strcat(json, "\n]\n}");
    return json;
}

bool split(char * input, TerminalDevice* info) {
    char* token;
    token = strtok(input, ",");
    if (token != NULL) {
        if (info->code != atou(token)) {
            return false;
        }
    }
    token = strtok(NULL, ",");
    if (token != NULL) {
		info->latit = atof(token);
    }
    token = strtok(NULL, ",");
    if (token != NULL) {
		info->longit = atof(token);
    }
    token = strtok(NULL, ",");
    if (token != NULL) {
        if (MY_ID_INT != atou(token)) {
            return false;
        }
    }
    return true;
}

void load_array(unsigned int* array, size_t max_size) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(FLASH_STORAGE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        ESP_LOGW("device", "NVS open failed or not initialized. Starting fresh.");
        device_count = 0;
        return;
    }

    size_t required_size = max_size * sizeof(unsigned int);
    err = nvs_get_blob(handle, "device_struct", array, &required_size);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI("device", "No existing device list found.");
        device_count = 0;
    } else if (err == ESP_OK) {
        device_count = required_size / sizeof(unsigned int);
        ESP_LOGI("device", "Loaded %d devices from NVS.", device_count);
    } else {
        ESP_LOGE("device", "Error reading NVS: %s", esp_err_to_name(err));
        device_count = 0;
    }

    nvs_close(handle);
}

void save_array(const unsigned int* array, size_t count) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(FLASH_STORAGE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE("device", "Failed to open NVS for writing: %s", esp_err_to_name(err));
        return;
    }

    err = nvs_set_blob(handle, "device_struct", array, sizeof(unsigned int) * count);
    if (err != ESP_OK) {
        ESP_LOGE("device", "Failed to write device list: %s", esp_err_to_name(err));
    } else {
        nvs_commit(handle);
        ESP_LOGI("device", "Saved %d devices to NVS.", count);
    }

    nvs_close(handle);
}

bool device_add(unsigned int device_id) {
    for (size_t i = 0; i < device_count; i++) {
        if (device_list[i] == device_id) {
            ESP_LOGW("device", "Device ID %u already exists. Skipping.", device_id);
            return false;
        }
    }

    if (device_count >= MAX_DEVICE_CNT) {
        ESP_LOGE("device", "Device list full. Cannot add ID %u.", device_id);
        return false;
    }

    device_list[device_count++] = device_id;
    save_array(device_list, device_count);
    ESP_LOGI("device", "Device ID %u added.", device_id);
    return true;
}


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


void lora_init() {
    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));


    lora_write_register(0x01, 0x80); 
    lora_write_register(0x01, 0x81); 


    uint64_t frf = ((uint64_t)LORA_FREQUENCY << 19) / 32000000;
    lora_write_register(0x06, (frf >> 16) & 0xFF);
    lora_write_register(0x07, (frf >> 8) & 0xFF);
    lora_write_register(0x08, (frf >> 0) & 0xFF);


    lora_write_register(0x1D, 0x72);
    lora_write_register(0x1E, 0xA4); 


    lora_write_register(0x20, 0x00);
    lora_write_register(0x21, 0x08);


    lora_write_register(0x1F, 0x00); 
}


void lora_send_packet(const char* data) {
    size_t len = strlen(data);

    lora_write_register(0x01, 0x81);


    lora_write_register(0x0E, 0x00);
    lora_write_register(0x0D, 0x00);

    for (int i = 0; i < len; i++) {
        lora_write_register(0x00, data[i]);
    }

    lora_write_register(0x22, len);
    lora_write_register(0x01, 0x83);


    while ((lora_read_register(0x12) & 0x08) == 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    lora_write_register(0x12, 0xFF);
}

int lora_receive_packet(char *buf) {
    int len = 0;
    
    uint8_t irq_flags = lora_read_register(0x12);

    if (irq_flags & 0x40) {
        len = lora_read_register(0x13);

        if (len > LORA_MAX_PACKET_SIZE) {
            len = LORA_MAX_PACKET_SIZE;
        }

        uint8_t fifo_addr = lora_read_register(0x10);
        lora_write_register(0x0D, fifo_addr);

        for (int i = 0; i < len; i++) {
            buf[i] = lora_read_register(0x00);
        }

        lora_write_register(0x12, 0xFF);
    }

    return len;
}

esp_err_t init_nvs_with_handling(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG_NVS, "NVS 문제가 발생했습니다. 플래시를 지우고 다시 초기화합니다.");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG_NVS, "NVS 초기화 실패: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG_NVS, "NVS 초기화 성공");
    }

    return ret;
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
        if (gps_data_all_receive) {
            char* server_send_string_JSON = make_json_payload();
            if (server_send_string_JSON != NULL) {
                size_t len = strlen(server_send_string_JSON);
                int sent = send(sock, server_send_string_JSON, len, 0);
                if (sent < 0) {
                    ESP_LOGE("TCP", "Error occurred during sending: errno %d", errno);
                } else {
                    ESP_LOGI("TCP", "Sent %d bytes", sent);
                }
                free(server_send_string_JSON);
            } else {
                ESP_LOGE("TCP", "Failed to allocate JSON payload");
            }
            gps_data_all_receive = false;
        }

        int r = recv(sock, rx_buffer, sizeof(rx_buffer)-1, MSG_DONTWAIT);
        if (r > 0) {
            rx_buffer[r] = '\0';
            ESP_LOGI("TCP", "Received: %s", rx_buffer);
            
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    close(sock);
    vTaskDelete(NULL);
}


void lora_host_task(void *pvParameters){
	for(int i = 0 ;i <device_count;i++){
		TerminalDevice* new_device = (TerminalDevice*)malloc(sizeof(TerminalDevice));
		
		new_device->code = device_list[i];
		device_info[i] = new_device;
	}
	
	while(1){
		for(int i = 0; i < device_count;i++){
			char gps_get_messege[17];
			sprintf(gps_get_messege,"%u", device_info[i]->code);
			strcat(gps_get_messege, "getgps");
			lora_send_packet(gps_get_messege);
			
			char buf[LORA_MAX_PACKET_SIZE];
			int packet_len;
			while(1){
				packet_len = lora_receive_packet(buf);
				if(packet_len < 0){
					if(strstr(buf, MY_ID_CHAR)){
						split(buf, device_info[i]);
						break;
					}
				}
			}
			
			
		}
		gps_data_all_receive = true;
	}
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

    ESP_ERROR_CHECK(spi_bus_initialize(LORA_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(LORA_SPI_HOST, &devcfg, &lora_spi));
	ESP_ERROR_CHECK(init_nvs_with_handling());
	
	esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    load_array(device_list, MAX_DEVICE_CNT);
	
	gpio_num_t pins[] = {CTRL, FIND_HOST, Enter_AND_SPACE, UP,DOWN};
	for (int i = 0; i < sizeof(pins)/sizeof(pins[0]); i++) {
	    gpio_set_direction(pins[i], GPIO_MODE_INPUT);
	    gpio_set_pull_mode(pins[i], GPIO_FLOATING);
	}
	
	wifi_init_sta();
	lora_init();
	
	xTaskCreate(tcp_client_task, "tcp_client", 4096, NULL, 5, NULL);
	xTaskCreate(lora_host_task, "LoRa_host", 4096, NULL, 5, NULL);
    while (1) {
		if(gpio_get_level(CTRL) == 1 && gpio_get_level(FIND_HOST) == 1){
			char buf[LORA_MAX_PACKET_SIZE];
			int packet_len;
			while(1){
				memset(buf, 0, sizeof(buf));
				packet_len = lora_receive_packet(buf);
				
				if(packet_len > 0){
					if (strstr(buf, "FINDHOST") != NULL){
						char temp[10];
						for(int i = 0 ;i < 10;i++){
							temp[i] = buf[i];
						}
		                device_add(atoi(temp));
		                break;
		            }
				}
				if(gpio_get_level(CTRL) == 1 && gpio_get_level(DOWN) == 1){
					break;
				}
			}
		}
    }
}