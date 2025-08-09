#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "arch/sys_arch.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "hal/gpio_types.h"
#include "lwip/sockets.h"
#include "soc/gpio_num.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_netif_ip_addr.h"
#include "driver/spi_master.h"
#include "cJSON.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "esp_websocket_client.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))

#define HOST_IP_ADDR "https://flp24.com"
#define PORT "443"
#define WIFI_CONNECTED_BIT BIT0
#define DNS_PORT 53
#define DNS_MAX_LEN 256

#define PIN_NUM_MISO GPIO_NUM_19
#define PIN_NUM_MOSI GPIO_NUM_23
#define PIN_NUM_CLK  GPIO_NUM_5
#define PIN_NUM_CS   GPIO_NUM_18
#define PIN_NUM_RST  GPIO_NUM_16
#define PIN_NUM_IRQ  GPIO_NUM_26
#define GPS_RX       GPIO_NUM_33
#define GPS_TX       GPIO_NUM_32
#define CTRL         GPIO_NUM_17
#define FIND_HOST    GPIO_NUM_25
#define ENTER_SPACE  GPIO_NUM_27
#define BTN_UP       GPIO_NUM_34
#define BTN_DOWN     GPIO_NUM_35

#define LORA_SPI_HOST HSPI_HOST
#define LORA_FREQUENCY 433000000
#define LORA_MAX_PACKET_SIZE 256
#define MAX_DEVICE_CNT 50
#define FLASH_STORAGE "storage"
#define UART_GPS UART_NUM_1
#define BUF_SIZE 1024

#define TXD_PIN GPIO_NUM_33
#define RXD_PIN GPIO_NUM_32

static const char *tag_gps = "GPS";
static const char *tag_wss = "WSS_CLIENT";
static const char *tag_wifi = "WIFI";
static const char *tag_nvs = "NVS";
static const char *tag_softap = "SOFT_AP";

static double latitude = 0.0;
static double longitude = 0.0;

static EventGroupHandle_t wifi_event_group;
esp_websocket_client_handle_t websocket_client = NULL;

static const int my_id_int = 1726384219;
static const char *my_id_char = "1726384219";

static char wifi_ssid[32] = "Your_SSID";
static char wifi_password[64] = "Your_PASSWORD";
static char device_ssid[32] = "FLP_HOST";
static char device_password[64] = "Defelt";

static int rx_id = 0;
static int rx_danger = 0;
static bool gps_data_all_receive = false;

static spi_device_handle_t lora_spi;
static httpd_handle_t server = NULL;

typedef struct TerminalDevice {
    int Code;
    double Latit;
    double Longit;
    uint8_t Danger;
} TerminalDevice;

static int device_list[MAX_DEVICE_CNT];
static TerminalDevice *device_info[MAX_DEVICE_CNT];
static size_t device_count = 0;


char* make_json_payload(void) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "hostId", my_id_char);

    cJSON *members_array = cJSON_CreateArray();

    for (int i = 0; i < MAX_DEVICE_CNT; i++) {
        if (!device_info[i]) continue;

        cJSON *member = cJSON_CreateObject();

        cJSON_AddNumberToObject(member, "memberId", device_info[i]->Code);
        cJSON_AddNumberToObject(member, "lat", device_info[i]->Latit);
        cJSON_AddNumberToObject(member, "lon", device_info[i]->Longit);

        cJSON_AddItemToArray(members_array, member);
    }

    cJSON_AddItemToObject(root, "members", members_array);

    char *json_str = cJSON_PrintUnformatted(root);

    cJSON_Delete(root);
    return json_str;
}

void parse_rx_buffer(const char *rx_buffer) {
    cJSON *root = cJSON_Parse(rx_buffer);
    if (root == NULL) {
        ESP_LOGE("JSON", "Failed to parse rx_buffer");
        return;
    }

    cJSON *id_item = cJSON_GetObjectItem(root, "id");
    cJSON *distance_item = cJSON_GetObjectItem(root, "distance");

    if (cJSON_IsNumber(id_item) && cJSON_IsNumber(distance_item)) {
        rx_id = id_item->valueint;
        rx_danger = distance_item->valueint;

        ESP_LOGI("JSON", "Parsed id: %d, distance: %d", rx_id, rx_danger);
    } else {
        ESP_LOGE("JSON", "Invalid JSON format");
    }

    cJSON_Delete(root);
}

bool split(char * input, TerminalDevice* info) {
    char* token;
    token = strtok(input, ",");
    if (token != NULL) {
        if (info->Code != atoi(token)) {
            return false;
        }
    }
    token = strtok(NULL, ",");
    if (token != NULL) {
		info->Latit = atof(token);
    }
    token = strtok(NULL, ",");
    if (token != NULL) {
		info->Longit = atof(token);
    }
    token = strtok(NULL, ",");
    if (token != NULL) {
        if (my_id_int != atoi(token)) {
            return false;
        }
    }
    return true;
}

void load_array(int* array, size_t max_size) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open("device_info", NVS_READONLY, &handle);
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

void load_basic_info() {
    nvs_handle_t handle;
    esp_err_t err = nvs_open("basic_info", NVS_READONLY, &handle);
    if (err != ESP_OK) {
        ESP_LOGW("basic_info", "NVS open failed or not initialized. Starting fresh.");
        strcpy(wifi_ssid, "None");
        strcpy(wifi_password, "None");
        strcpy(device_ssid, "FLP_host");
        strcpy(device_password, "Default");
        return;
    }

    size_t SSID_s = 32;
    size_t PASS_s = 64;

    err = nvs_get_blob(handle, "Device_SSID", device_ssid, &SSID_s);
	if (err == ESP_ERR_NVS_NOT_FOUND) {
	    ESP_LOGW("basic_info", "Device_SSID not found. Using default.");
	    strcpy(device_ssid, "FLP_host");
	} else if (err != ESP_OK) {
	    ESP_LOGE("basic_info", "Error reading Device_SSID: %s", esp_err_to_name(err));
	}
	
	err = nvs_get_blob(handle, "Wifi_SSID", wifi_ssid, &SSID_s);
	if (err == ESP_ERR_NVS_NOT_FOUND) {
	    ESP_LOGW("basic_info", "Wifi_SSID not found. Using default.");
	    strcpy(wifi_ssid, "None");
	} else if (err != ESP_OK) {
	    ESP_LOGE("basic_info", "Error reading Wifi_SSID: %s", esp_err_to_name(err));
	}
	
	err = nvs_get_blob(handle, "Device_pass", device_password, &PASS_s);
	if (err == ESP_ERR_NVS_NOT_FOUND) {
	    ESP_LOGW("basic_info", "Device_pass not found. Using default.");
	    strcpy(device_password, "Default");
	} else if (err != ESP_OK) {
	    ESP_LOGE("basic_info", "Error reading Device_pass: %s", esp_err_to_name(err));
	}
	
	err = nvs_get_blob(handle, "Wifi_pass", wifi_password, &PASS_s);
	if (err == ESP_ERR_NVS_NOT_FOUND) {
	    ESP_LOGW("basic_info", "Wifi_pass not found. Using default.");
	    strcpy(wifi_password, "None");
	} else if (err != ESP_OK) {
	    ESP_LOGE("basic_info", "Error reading Wifi_pass: %s", esp_err_to_name(err));
	}
	
	nvs_close(handle);
}

void save_array(const int* array, size_t count) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open("device_info", NVS_READWRITE, &handle);
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

bool device_add(int device_id) {
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

double convert_to_decimal_degrees(const char* nmea_coord, char direction) {
    double raw = atof(nmea_coord);
    int degrees = (int)(raw / 100);
    double minutes = raw - (degrees * 100);
    double decimal_degrees = degrees + (minutes / 60.0);

    if (direction == 'S' || direction == 'W') {
        decimal_degrees *= -1.0;
    }

    return decimal_degrees;
}

void parse_nmea_sentence(char* sentence) {
    if (strstr(sentence, "$GPGGA") || strstr(sentence, "$GPRMC")) {
        char *token;
        int field_index = 0;
        char lat[16] = {0}, lon[16] = {0};
        char lat_dir = 0, lon_dir = 0;

        token = strtok(sentence, ",");

        while (token != NULL) {
            field_index++;

            if (field_index == 3) strncpy(lat, token, sizeof(lat));
            if (field_index == 4) lat_dir = token[0];
            if (field_index == 5) strncpy(lon, token, sizeof(lon));
            if (field_index == 6) lon_dir = token[0];

            token = strtok(NULL, ",");
        }

        if (strlen(lat) > 0 && strlen(lon) > 0) {
            latitude = convert_to_decimal_degrees(lat, lat_dir);
            longitude = convert_to_decimal_degrees(lon, lon_dir);
            ESP_LOGI(tag_gps, "위도: %lf, 경도: %lf", latitude, longitude);
        }
    }
}

void gps_uart_init() {
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_driver_install(UART_GPS, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_GPS, &uart_config);
    uart_set_pin(UART_GPS, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void gps_task(void *arg) {
    gps_uart_init();

    uint8_t* data = (uint8_t*) malloc(BUF_SIZE);
    while (1) {
        int len = uart_read_bytes(UART_GPS, data, BUF_SIZE - 1, pdMS_TO_TICKS(1000));
        if (len > 0) {
            data[len] = '\0';
            char* line = strtok((char*)data, "\r\n");
            while (line != NULL) {
                parse_nmea_sentence(line);
                line = strtok(NULL, "\r\n");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    free(data);
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
        ESP_LOGW(tag_nvs, "NVS 문제가 발생했습니다. 플래시를 지우고 다시 초기화합니다.");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    if (ret != ESP_OK) {
        ESP_LOGE(tag_nvs, "NVS 초기화 실패: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(tag_nvs, "NVS 초기화 성공");
    }

    return ret;
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
        case WIFI_EVENT_STA_START:
            esp_wifi_connect();
            ESP_LOGI(tag_wifi, "STA 시작됨. 연결 시도 중...");
            break;

        case WIFI_EVENT_STA_DISCONNECTED:
		    esp_wifi_connect();
		    ESP_LOGI(tag_wifi, "STA 연결 끊김. 재연결 시도...");
		    break;

        case WIFI_EVENT_AP_START:
            ESP_LOGI(tag_wifi, "SoftAP 시작됨.");
            break;
        
        default:
            break;
        }
    }

    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        char ip_str[INET_ADDRSTRLEN];
        esp_ip4addr_ntoa(&event->ip_info.ip, ip_str, INET_ADDRSTRLEN);
        ESP_LOGI(tag_wifi, "STA에서 IP 할당됨: %s", ip_str);
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}



void wifi_init_softap_sta(void) {
    esp_netif_init();
    esp_event_loop_create_default();

    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    // Register event handlers
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL);

    wifi_config_t wifi_sta_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    strncpy((char *)wifi_sta_config.sta.ssid, (char *)wifi_ssid, sizeof(wifi_sta_config.sta.ssid));
    strncpy((char *)wifi_sta_config.sta.password, (char *)wifi_password, sizeof(wifi_sta_config.sta.password));

    wifi_config_t wifi_ap_config = {
        .ap = {
            .ssid_len = sizeof(wifi_ap_config.ap.password),
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    strncpy((char *)wifi_ap_config.ap.ssid, (char *)device_ssid, sizeof(wifi_ap_config.ap.ssid));
    strncpy((char *)wifi_ap_config.ap.password, (char *)device_password, sizeof(wifi_ap_config.ap.password)); // @suppress("Field cannot be resolved")

    if (strlen((char *)wifi_ap_config.ap.password) == 0) {
        wifi_ap_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    esp_wifi_set_mode(WIFI_MODE_APSTA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_sta_config);
    esp_wifi_set_config(WIFI_IF_AP, &wifi_ap_config);
    esp_wifi_start();
}

void save_basic_info() {
    nvs_handle_t handle;
    esp_err_t err;

    err = nvs_open("basic_info", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE("device", "Failed to open NVS for writing: %s", esp_err_to_name(err));
        return;
    }

    err = nvs_set_blob(handle, "Device_SSID", device_ssid, 32);
    if (err != ESP_OK) {
        ESP_LOGE("device", "Failed to write Device_SSID: %s", esp_err_to_name(err));
    }

    err = nvs_set_blob(handle, "Device_pass", device_password, 64);
    if (err != ESP_OK) {
        ESP_LOGE("device", "Failed to write Device_pass: %s", esp_err_to_name(err));
    }

    err = nvs_set_blob(handle, "Wifi_SSID", wifi_ssid, 32);
    if (err != ESP_OK) {
        ESP_LOGE("wifi", "Failed to write Wifi_SSID: %s", esp_err_to_name(err));
    }

    err = nvs_set_blob(handle, "Wifi_pass", wifi_password, 64);
    if (err != ESP_OK) {
        ESP_LOGE("wifi", "Failed to write Wifi_pass: %s", esp_err_to_name(err));
    }

    err = nvs_commit(handle);
    if (err == ESP_OK) {
        ESP_LOGI("device", "All data saved successfully to NVS.");
    } else {
        ESP_LOGE("device", "Failed to commit NVS: %s", esp_err_to_name(err));
    }

    nvs_close(handle);
}


static void dns_server_task(void *pvParameters) {
    uint8_t buffer[DNS_MAX_LEN];
    struct sockaddr_in client_addr;
    socklen_t client_addr_len = sizeof(client_addr);
    struct sockaddr_in server_addr;

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE("DNS", "Socket 생성 실패");
        vTaskDelete(NULL);
    }

    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(DNS_PORT);

    if (bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE("DNS", "Bind 실패");
        close(sock);
        vTaskDelete(NULL);
    }

    ESP_LOGI("DNS", "DNS 서버가 시작되었습니다.");

    while (1) {
        int len = recvfrom(sock, buffer, sizeof(buffer), 0, (struct sockaddr *)&client_addr, &client_addr_len);
        if (len > 0) {
            // DNS 응답 헤더 설정
            buffer[2] |= 0x80;
            buffer[2] |= 0x04;
            buffer[3] |= 0x80;

            // esp_netif API를 사용하여 AP의 IP 정보 가져오기
            esp_netif_t *ap_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
            esp_netif_ip_info_t ip_info;
            esp_netif_get_ip_info(ap_netif, &ip_info);

            // DNS 응답 레코드 생성
            uint8_t *dns_response = buffer + len;
            *dns_response++ = 0xC0;
            *dns_response++ = 0x0C;
            *dns_response++ = 0x00;
            *dns_response++ = 0x01; // 타입 A
            *dns_response++ = 0x00;
            *dns_response++ = 0x01; // 클래스 IN
            *dns_response++ = 0x00;
            *dns_response++ = 0x00;
            *dns_response++ = 0x00;
            *dns_response++ = 0x78; // TTL
            *dns_response++ = 0x00;
            *dns_response++ = 0x04; // 데이터 길이

            // IP 주소 추가
            uint32_t ip_addr = ip_info.ip.addr;
            *dns_response++ = (uint8_t)(ip_addr >> 0);
            *dns_response++ = (uint8_t)(ip_addr >> 8);
            *dns_response++ = (uint8_t)(ip_addr >> 16);
            *dns_response++ = (uint8_t)(ip_addr >> 24);

            len += 16;
            sendto(sock, buffer, len, 0, (struct sockaddr *)&client_addr, client_addr_len);
        }
    }
    close(sock);
    vTaskDelete(NULL);
}


// POST 요청을 처리하는 핸들러 (기존과 동일)
static esp_err_t submit_post_handler(httpd_req_t *req) {
    char buf[256];
    int ret, remaining = req->content_len;

    char sta_ssid_input[32] = {0};
    char sta_pass_input[64] = {0};
    char ap_ssid_input[32] = {0};
    char ap_pass_input[64] = {0};

    while (remaining > 0) {
        if ((ret = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf) - 1))) <= 0) {
            return ESP_FAIL;
        }
        remaining -= ret;
        buf[ret] = '\0';

        char *ssid_ptr = strstr(buf, "SSID=");
        if (ssid_ptr) sscanf(ssid_ptr + 5, "%31[^&]", sta_ssid_input);

        char *pass_ptr = strstr(buf, "Wifipass=");
        if (pass_ptr) sscanf(pass_ptr + 9, "%63[^&]", sta_pass_input);

        char *ap_ssid_ptr = strstr(buf, "Device_SSID=");
        if (ap_ssid_ptr) sscanf(ap_ssid_ptr + 12, "%31[^&]", ap_ssid_input);

        char *ap_pass_ptr = strstr(buf, "Device_Wifipass=");
        if (ap_pass_ptr) sscanf(ap_pass_ptr + 17, "%63[^&]", ap_pass_input);
    }

    if (strlen(sta_ssid_input) > 0) { // 비밀번호는 비어있을 수 있음
        strncpy(wifi_ssid, sta_ssid_input, sizeof(wifi_ssid));
        strncpy(wifi_password, sta_pass_input, sizeof(wifi_password));
    }

    if (strlen(ap_ssid_input) > 0) { // 비밀번호는 비어있을 수 있음
        strncpy(device_ssid, ap_ssid_input, sizeof(device_ssid));
        strncpy(device_password, ap_pass_input, sizeof(device_password));
    }

    // 변경된 정보를 NVS에 저장
    save_basic_info();

    // Wi-Fi 재설정 및 재연결 로직
    ESP_LOGI(tag_wifi, "Wi-Fi settings updated. Restarting Wi-Fi...");
    esp_wifi_disconnect();

    wifi_config_t wifi_sta_config = { .sta = { .threshold.authmode = WIFI_AUTH_WPA2_PSK }};
    strncpy((char *)wifi_sta_config.sta.ssid, wifi_ssid, sizeof(wifi_sta_config.sta.ssid));
    strncpy((char *)wifi_sta_config.sta.password, wifi_password, sizeof(wifi_sta_config.sta.password));
    esp_wifi_set_config(WIFI_IF_STA, &wifi_sta_config);

    wifi_config_t wifi_ap_config = { .ap = { .max_connection = 4, .authmode = WIFI_AUTH_WPA_WPA2_PSK }};
    strncpy((char *)wifi_ap_config.ap.ssid, device_ssid, sizeof(wifi_ap_config.ap.ssid));
    strncpy((char *)wifi_ap_config.ap.password, device_password, sizeof(wifi_ap_config.ap.password));
    if (strlen((char *)wifi_ap_config.ap.password) == 0) {
        wifi_ap_config.ap.authmode = WIFI_AUTH_OPEN;
    }
    esp_wifi_set_config(WIFI_IF_AP, &wifi_ap_config);

    esp_wifi_connect();

    // 사용자에게 성공 메시지 표시
    const char* resp_str = "<h2>Settings Updated!</h2><p>The device will now connect to the new network. You may need to reconnect to this device's Wi-Fi if you changed its settings.</p>";
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

// 캡티브 포털의 메인 페이지를 보여주는 핸들러
static esp_err_t captive_portal_get_handler(httpd_req_t *req) {
    // 여기에 기존의 HTML 페이지 생성 로직을 넣습니다.
    // (root_get_handler의 내용을 그대로 가져옵니다)
    wifi_config_t sta_config;
    wifi_config_t ap_config;

    esp_wifi_get_config(WIFI_IF_STA, &sta_config);
    esp_wifi_get_config(WIFI_IF_AP, &ap_config);

    // html_template는 이전에 개선한 버전의 문자열을 사용
    const char *html_template =
            "<!DOCTYPE html>"
            "<html>"
            "<head>"
            "    <title>Host Information</title>"
            "    <meta http-equiv=\"refresh\" content=\"5\">"
            "    <style>"
            "        body { font-family: sans-serif; background-color: #f4f4f4; padding: 20px; }"
            "        fieldset { border: 1px solid #ccc; border-radius: 8px; margin-bottom: 20px; padding: 20px; background-color: #fff; }"
            "        legend { font-weight: bold; font-size: 1.2em; color: #333; }"
            "        table { border-collapse: collapse; background-color: #e0e0e0; width: 100%; }"
            "        th, td { border: 1px solid #999; padding: 8px; text-align: left; }"
            "        th { background-color: #d0d0d0; }"
            "        input[type=\"text\"] { width: calc(100% - 20px); padding: 8px; margin-bottom: 10px; border: 1px solid #ccc; border-radius: 4px; }"
            "        button { padding: 10px 15px; border: none; background-color: #007bff; color: white; border-radius: 4px; cursor: pointer; }"
            "        button:hover { background-color: #0056b3; }"
            "    </style>"
            "</head>"
            "<body>"
            "    <fieldset>"
            "        <legend>Host Info</legend>"
            "        <table>"
            "            <tr><th>Category</th><th>Data</th></tr>"
            "            <tr><td>Connecting SSID</td><td>%s</td></tr>"
            "            <tr><td>Connecting Password</td><td>%s</td></tr>"
            "            <tr><td>Device SSID</td><td>%s</td></tr>"
            "            <tr><td>Device Password</td><td>%s</td></tr>"
            "            <tr><td>Current Connecting Device</td><td>%d client(s)</td></tr>"
            "            <tr><td>Status Connection Server</td><td>%s</td></tr>"
            "            <tr><td>Device Status</td><td>%s</td></tr>"
            "        </table>"
            "    </fieldset>"
            "    <form method=\"post\" action=\"/submit\">"
            "        <fieldset>"
            "            <legend>Wifi SSID/Password Change</legend>"
            "            <p>SSID: <input name=\"SSID\" type=\"text\" /></p>"
            "            <p>Password: <input name=\"Wifipass\" type=\"text\" /></p>"
            "            <button type=\"submit\">Submit</button>"
            "        </fieldset>"
            "    </form>"
            "    <form method=\"post\" action=\"/submit\">"
            "        <fieldset>"
            "            <legend>Device SSID/Password Change</legend>"
            "            <p>SSID: <input name=\"Device_SSID\" type=\"text\" /></p>"
            "            <p>Password: <input name=\"Device_Wifipass\" type=\"text\" /></p>"
            "            <button type=\"submit\">Submit</button>"
            "        </fieldset>"
            "    </form>"
            "</body>"
            "</html>";

    char html_buffer[4096];

    wifi_sta_list_t sta_list;
    esp_wifi_ap_get_sta_list(&sta_list);

    const char *server_status = esp_websocket_client_is_connected(websocket_client) ? "Connected" : "Disconnected";

    char device_status_str[MAX_DEVICE_CNT * 12] = "No devices registered";
    if (device_count > 0) {
    	char temp_str[256] = {0};
		for (int i = 0; i < device_count; i++) {
			char id_str[12];
			sprintf(id_str, "%d, ", device_list[i]);
			strcat(temp_str, id_str);
		}
		// 마지막 쉼표와 공백 제거
		temp_str[strlen(temp_str) - 2] = '\0';
		strcpy(device_status_str, temp_str);
    }

    snprintf(html_buffer, sizeof(html_buffer), html_template,
        (char *)sta_config.sta.ssid, (char *)sta_config.sta.password,
        (char *)ap_config.ap.ssid, (char *)ap_config.ap.password,
        sta_list.num, server_status, device_status_str
    );

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html_buffer, HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

// 웹 서버 시작 함수 (캡티브 포털용으로 수정)
void start_webserver(void) {
    if (server != NULL) {
        return; // 이미 실행 중이면 아무것도 하지 않음
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    // URI 매칭이 없을 때 모든 요청을 처리할 핸들러 등록
    config.uri_match_fn = httpd_uri_match_wildcard;

    ESP_LOGI(tag_softap, "Starting webserver for captive portal");
    if (httpd_start(&server, &config) == ESP_OK) {
        // POST 요청을 처리할 URI
        httpd_uri_t submit_uri = {
            .uri       = "/submit",
            .method    = HTTP_POST,
            .handler   = submit_post_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &submit_uri);

        // 그 외 모든 GET 요청을 처리할 URI (와일드카드 사용)
        httpd_uri_t captive_portal_uri = {
            .uri       = "/*", // 모든 경로를 의미
            .method    = HTTP_GET,
            .handler   = captive_portal_get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &captive_portal_uri);
    } else {
        ESP_LOGE(tag_softap, "Failed to start webserver");
    }
}


void start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = root_get_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &root);

        httpd_uri_t submit = {
            .uri = "/submit",
            .method = HTTP_POST,
            .handler = submit_post_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &submit);
    }
}

static void websocket_event_handler(void *handler_args, esp_event_base_t base,
                                    int32_t event_id, void *event_data) {
    esp_websocket_event_data_t *event = (esp_websocket_event_data_t *)event_data;

    switch (event_id) {
        case WEBSOCKET_EVENT_CONNECTED:
            ESP_LOGI(tag_wss, "WebSocket connected");
            break;
        case WEBSOCKET_EVENT_DISCONNECTED:
            ESP_LOGI(tag_wss, "WebSocket disconnected");
            break;
        case WEBSOCKET_EVENT_DATA:
            ESP_LOGI(tag_wss, "Received data: %.*s", event->data_len, (char *)event->data_ptr);

            char *rx_data = strndup((char *)event->data_ptr, event->data_len);
            if (rx_data) {
                cJSON *root = cJSON_Parse(rx_data);
                if (root) {
                    cJSON *danger_array = cJSON_GetObjectItem(root, "danger");
                    if (cJSON_IsArray(danger_array)) {
                        int count = cJSON_GetArraySize(danger_array);
                        for (int i = 0; i < count && i < MAX_DEVICE_CNT; i++) {
                            cJSON *item = cJSON_GetArrayItem(danger_array, i);
                            cJSON *id = cJSON_GetObjectItem(item, "id");
                            cJSON *distance = cJSON_GetObjectItem(item, "distance");

                            if (cJSON_IsNumber(id) && cJSON_IsNumber(distance)) {
                                for (int j = 0; j < device_count; j++) {
                                    if (device_info[j] && device_info[j]->Code == id->valueint) {
                                        device_info[j]->Danger = distance->valueint;
                                    }
                                }
                            }
                        }
                    }
                    cJSON_Delete(root);
                }
                free(rx_data);
            }
            break;
        case WEBSOCKET_EVENT_ERROR:
            ESP_LOGE(tag_wss, "WebSocket error occurred");
            break;
    }
}

void wss_client_task(void *pv_parameters) {
    esp_websocket_client_config_t websocket_cfg = {
        .uri = "wss://" HOST_IP_ADDR ":" PORT "/",
        .network_timeout_ms = 10000,
    };
    esp_websocket_client_handle_t client = esp_websocket_client_init(&websocket_cfg);
    esp_websocket_register_events(client, WEBSOCKET_EVENT_ANY, websocket_event_handler, (void *)client);
    while (1) {

        if (esp_websocket_client_is_connected(client)) {

            if (gps_data_all_receive) {
                char *json_payload = make_json_payload();
                if (json_payload != NULL) {
                    ESP_LOGI(tag_wss, "Attempting to send JSON payload...");
                    int ret = esp_websocket_client_send_text(client, json_payload, strlen(json_payload), portMAX_DELAY);
                    if (ret > 0) {
                        ESP_LOGI(tag_wss, "Sent JSON payload successfully");
                    } else {
                        ESP_LOGE(tag_wss, "Failed to send payload, error code: %d", ret);
                    }
                    free(json_payload);
                }
                gps_data_all_receive = false;
            }
        }

        else {
            EventBits_t bits = xEventGroupGetBits(wifi_event_group);
            if (bits & WIFI_CONNECTED_BIT) {
                ESP_LOGI(tag_wss, "WebSocket disconnected. Attempting to reconnect...");
                esp_websocket_client_start(client); // 재연결 시도
            } else {
                ESP_LOGW(tag_wss, "Wi-Fi not connected. Waiting for Wi-Fi connection...");
            }
            vTaskDelay(pdMS_TO_TICKS(5000));
        }

        // 루프 주기
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // 실제로는 도달하지 않지만, 만약을 위한 정리 코드
    esp_websocket_client_stop(client);
    esp_websocket_client_destroy(client);
    vTaskDelete(NULL);
}

void lora_host_task(void *pv_parameters) {
    const TickType_t timeout_ticks = pdMS_TO_TICKS(5000);

    for (int i = 0; i < device_count; i++) {
        TerminalDevice *new_device = malloc(sizeof(TerminalDevice));
        if (new_device) {
            new_device->Code = device_list[i];
            new_device->Danger = 0; // 위험도 초기화
            device_info[i] = new_device;
        }
    }

    while (1) {
        for (int i = 0; i < device_count; i++) {
            if (device_info[i] == NULL) continue;

            char gps_request_msg[33];
            snprintf(gps_request_msg, sizeof(gps_request_msg), "%u,%u,%u,getgps", device_info[i]->Code, my_id_int, device_info[i]->Danger);
            lora_send_packet(gps_request_msg);
            ESP_LOGI("LORA_HOST", "Sent to %d: %s", device_info[i]->Code, gps_request_msg);

            TickType_t start_time = xTaskGetTickCount();
            bool received = false;

            char buf[LORA_MAX_PACKET_SIZE];
            int packet_len = 0;

            while ((xTaskGetTickCount() - start_time) < timeout_ticks) {
                packet_len = lora_receive_packet(buf);
                if (packet_len > 0) {
                    if (strstr(buf, my_id_char) != NULL) {
                        if (split(buf, device_info[i])) {
                            ESP_LOGI("LORA_HOST", "Received from %d: %s", device_info[i]->Code, buf);
                            received = true;
                            break;
                        }
                    }
                }
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            if (!received) {
                ESP_LOGW("LORA_HOST", "Timeout waiting for response from device %d", device_info[i]->Code);
            }
        }

        gps_data_all_receive = true;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    for (int i = 0; i < device_count; i++) {
        if (device_info[i] != NULL) {
            free(device_info[i]);
        }
    }
    vTaskDelete(NULL);
}

void app_main(void) {
    spi_bus_config_t bus_config = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };

    spi_device_interface_config_t device_config = {
        .clock_speed_hz = 1 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 7
    };

    ESP_ERROR_CHECK(spi_bus_initialize(LORA_SPI_HOST, &bus_config, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(LORA_SPI_HOST, &device_config, &lora_spi));
    ESP_ERROR_CHECK(init_nvs_with_handling());

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    load_basic_info();
    load_array(device_list, MAX_DEVICE_CNT);

    gpio_num_t input_pins[] = {CTRL, FIND_HOST, ENTER_SPACE, BTN_UP, BTN_DOWN};
    for (int i = 0; i < sizeof(input_pins) / sizeof(input_pins[0]); i++) {
        gpio_set_direction(input_pins[i], GPIO_MODE_INPUT);
        gpio_set_pull_mode(input_pins[i], GPIO_FLOATING);
    }

    wifi_init_softap_sta();
    lora_init();
    start_webserver();

    xTaskCreatePinnedToCore(wss_client_task, "wss_client", 8192, NULL, 5, NULL, 1); 
    xTaskCreatePinnedToCore(lora_host_task, "lora_host", 4096, NULL, 5, NULL, 1);    
    xTaskCreatePinnedToCore(gps_task, "gps_task", 4096, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(dns_server_task, "dns_server", 4096, NULL, 5, NULL);

	while (1) {
		// CTRL과 FIND_HOST 버튼이 동시에 눌렸을 때만 등록 모드 진입
		if (gpio_get_level(CTRL) == 1 && gpio_get_level(FIND_HOST) == 1) {
			ESP_LOGI("HOST_MODE", "Entering device registration mode for 10 seconds...");

			// 타임아웃 설정 (10000ms = 10초)
			const TickType_t registration_timeout_ticks = pdMS_TO_TICKS(10000);
			TickType_t start_time = xTaskGetTickCount();

			char buf[LORA_MAX_PACKET_SIZE];
			int packet_len;

			// 10초 동안만 "FINDHOST" 패킷을 기다림
			while ((xTaskGetTickCount() - start_time) < registration_timeout_ticks) {
				memset(buf, 0, sizeof(buf));
				packet_len = lora_receive_packet(buf);

				if (packet_len > 0 && strstr(buf, "FINDHOST") != NULL) {
					int add_id;
					char *token;

					// "FINDHOST"가 포함된 패킷 파싱
					strtok(buf, ",");
					token = strtok(NULL, ",");

					if (token != NULL) {
						add_id = atoi(token);
						ESP_LOGI("HOST_MODE", "Received registration request from ID: %d", add_id);

						if (device_add(add_id)) {
							 ESP_LOGI("HOST_MODE", "Device %d added successfully", add_id);
							 // 등록 성공 시 lora_host_task 재시작 또는 동적 업데이트 필요
						} else {
							// 실패 메시지 전송 (필요 시)
							char fail_msg[32];
							snprintf(fail_msg, sizeof(fail_msg), "%d,host_set_fail", my_id_int);
							lora_send_packet(fail_msg);
							ESP_LOGW("HOST_MODE", "Device add failed for ID: %d", add_id);
						}
					} else {
						ESP_LOGW("HOST_MODE", "Invalid FINDHOST packet: missing ID");
					}

					// 하나의 장치를 처리한 후 등록 모드를 빠져나옴
					break;
				}
				vTaskDelay(pdMS_TO_TICKS(10)); // CPU 부담 감소
			}
			ESP_LOGI("HOST_MODE", "Exiting device registration mode.");
		}

		// 메인 루프는 다른 작업을 방해하지 않도록 주기적으로 딜레이를 가짐
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}

