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
#include "arch/sys_arch.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
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
#include "cJSON.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "esp_websocket_client.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))

#define HOST_IP_ADDR "192.168.0.100"
#define PORT "443"

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
#define UART_GPS UART_NUM_1
#define BUF_SIZE 1024

#define TXD_PIN GPIO_NUM_33  // GPS RX
#define RXD_PIN GPIO_NUM_32  // GPS TX

static const char *G_TAG = "GPS";
static const char *TAG = "WSS_CLIENT";
static const char *W_TAG = "wifi";

double latitude = 0.0;
double longitude = 0.0;

static EventGroupHandle_t wifi_event_group;
esp_websocket_client_handle_t client = NULL;

const int MY_ID_INT = 1726384219;
const char* MY_ID_CHAR = "1726384219";
char Wifi_SSID[32] = "Your_SSID";
char Wifi_password[64] = "Your_PASSWORD";
char Device_SSID[32] = "FLP_HOST";
char Device_password[64] = "Defelt";

int rx_id = 0;
int rx_danger = 0;
bool gps_data_all_receive = false;

spi_device_handle_t lora_spi;

typedef struct terminaldevice{
	int code;
	double latit;
	double longit;
	uint8_t danger;
} TerminalDevice;

int device_list[MAX_DEVICE_CNT];
TerminalDevice* device_info[MAX_DEVICE_CNT];
size_t device_count = 0;

char* make_json_payload(void) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "hostId", MY_ID_CHAR);

    cJSON *members_array = cJSON_CreateArray();

    for (int i = 0; i < MAX_DEVICE_CNT; i++) {
        if (!device_info[i]) continue;

        cJSON *member = cJSON_CreateObject();

        cJSON_AddNumberToObject(member, "memberId", device_info[i]->code);
        cJSON_AddNumberToObject(member, "lat", device_info[i]->latit);
        cJSON_AddNumberToObject(member, "lon", device_info[i]->longit);

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
        if (info->code != atoi(token)) {
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
        if (MY_ID_INT != atoi(token)) {
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
        strcpy(Wifi_SSID, "None");
        strcpy(Wifi_password, "None");
        strcpy(Device_SSID, "FLP_host");
        strcpy(Device_password, "Default");
        return;
    }

    size_t SSID_s = 32;
    size_t PASS_s = 64;

    err = nvs_get_blob(handle, "Device_SSID", Device_SSID, &SSID_s);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW("basic_info", "Device_SSID not found. Using default.");
        strcpy(Device_SSID, "FLP_host");
    } else if (err != ESP_OK) {
        ESP_LOGE("basic_info", "Error reading Device_SSID: %s", esp_err_to_name(err));
    }

    err = nvs_get_blob(handle, "Wifi_SSID", Wifi_SSID, &SSID_s);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW("basic_info", "Wifi_SSID not found. Using default.");
        strcpy(Wifi_SSID, "None");
    } else if (err != ESP_OK) {
        ESP_LOGE("basic_info", "Error reading Wifi_SSID: %s", esp_err_to_name(err));
    }

    err = nvs_get_blob(handle, "Device_pass", Device_password, &PASS_s);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW("basic_info", "Device_pass not found. Using default.");
        strcpy(Device_password, "Default");
    } else if (err != ESP_OK) {
        ESP_LOGE("basic_info", "Error reading Device_pass: %s", esp_err_to_name(err));
    }

    err = nvs_get_blob(handle, "Wifi_pass", Wifi_password, &PASS_s);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW("basic_info", "Wifi_pass not found. Using default.");
        strcpy(Wifi_password, "None");
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

// NMEA에서 DMM(Degree Decimal Minutes)을 DD(Decimal Degrees)로 변환
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
            ESP_LOGI(G_TAG, "위도: %lf, 경도: %lf", latitude, longitude);
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
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
        case WIFI_EVENT_STA_START:
            esp_wifi_connect();
            ESP_LOGI(W_TAG, "STA 시작됨. 연결 시도 중...");
            break;

        case WIFI_EVENT_STA_DISCONNECTED:
            if (client != NULL) {
		        esp_websocket_client_stop(client);
		        esp_websocket_client_destroy(client);
		        client = NULL;
		    }
		    esp_wifi_connect();
		    ESP_LOGI(W_TAG, "STA 연결 끊김. 재연결 시도...");
		    break;

        case WIFI_EVENT_AP_START:
            ESP_LOGI(W_TAG, "SoftAP 시작됨.");
            break;
        
        default:
            break;
        }
    }

    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        char ip_str[INET_ADDRSTRLEN];
        esp_ip4addr_ntoa(&event->ip_info.ip, ip_str, INET_ADDRSTRLEN);
        ESP_LOGI(W_TAG, "STA에서 IP 할당됨: %s", ip_str);
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}



void wifi_init_softap_sta(void){
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
    strncpy((char *)wifi_sta_config.sta.ssid, (char *)Wifi_SSID, sizeof(wifi_sta_config.sta.ssid));
	strncpy((char *)wifi_sta_config.sta.password, (char *)Wifi_password, sizeof(wifi_sta_config.sta.password));

    wifi_config_t wifi_ap_config = {
        .ap = {
            .ssid_len = sizeof(wifi_ap_config.ap.password),
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    strncpy((char *)wifi_ap_config.ap.ssid, (char *)Device_SSID, sizeof(wifi_ap_config.ap.ssid));
	strncpy((char *)wifi_ap_config.ap.password, (char *)Device_password, sizeof(wifi_ap_config.ap.password));

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

    err = nvs_set_blob(handle, "Device_SSID", Device_SSID, 32);
    if (err != ESP_OK) {
        ESP_LOGE("device", "Failed to write Device_SSID: %s", esp_err_to_name(err));
    }

    err = nvs_set_blob(handle, "Device_pass", Device_password, 64);
    if (err != ESP_OK) {
        ESP_LOGE("device", "Failed to write Device_pass: %s", esp_err_to_name(err));
    }

    err = nvs_set_blob(handle, "Wifi_SSID", Wifi_SSID, 32);
    if (err != ESP_OK) {
        ESP_LOGE("Wifi", "Failed to write Wifi_SSID: %s", esp_err_to_name(err));
    }

    err = nvs_set_blob(handle, "Wifi_pass", Wifi_password, 64);
    if (err != ESP_OK) {
        ESP_LOGE("Wifi", "Failed to write Wifi_pass: %s", esp_err_to_name(err));
    }

    err = nvs_commit(handle);
    if (err == ESP_OK) {
        ESP_LOGI("device", "All data saved successfully to NVS.");
    } else {
        ESP_LOGE("device", "Failed to commit NVS: %s", esp_err_to_name(err));
    }

    nvs_close(handle);
}

static esp_err_t root_get_handler(httpd_req_t *req) {
    wifi_config_t sta_config;
    wifi_config_t ap_config;

    esp_wifi_get_config(WIFI_IF_STA, &sta_config);
    esp_wifi_get_config(WIFI_IF_AP, &ap_config);

    char html_buffer[4096];

    const char *html_template = "<!DOCTYPE html>"
    "<html><body><head></head>"
    "<fieldset><legend>Host Info</legend>"
    "<table border=\"1px\" style=\"border-collapse: collapse; background-color: #e0e0e0\">"
    "<tr><th>category</th><th>data</th></tr>"
    "<tr><td>Connecting SSID</td><td>%s</td></tr>"
    "<tr><td>Connecting Password</td><td>%s</td></tr>"
    "<tr><td>Device SSID</td><td>%s</td></tr>"
    "<tr><td>Device Password</td><td>%s</td></tr>"
    "<tr><td>Current Connecting Device</td><td>%d client(s)</td></tr>"
    "<tr><td>Status Connection Server</td><td>%s</td></tr>"
    "<tr><td>Device Status</td><td>%s</td></tr>"
    "</table></fieldset>"
    "<form method=\"post\" action=\"/submit\">"
    "<fieldset><legend>Wifi SSID/Password change</legend>"
    "<p>SSID : <input name=\"SSID\" type=\"text\" /></p>"
    "<p>Password : <input name=\"Wifipass\" type=\"text\" /></p>"
    "<button type=\"submit\">submit</button></fieldset></form>"
    "<form method=\"post\" action=\"/submit\">"
    "<fieldset><legend>Device SSID/Password change</legend>"
    "<p>SSID : <input name=\"Device_SSID\" type=\"text\" /></p>"
    "<p>Password : <input name=\"Device_Wifipass\" type=\"text\" /></p>"
    "<button type=\"submit\">submit</button></fieldset></form>"
    "</body></html>";

    wifi_sta_list_t sta_list;
    esp_wifi_ap_get_sta_list(&sta_list);
    
    const char *server_status;

    if (esp_websocket_client_is_connected(client)) {
	    server_status = "Connected";
	} else {
	    server_status = "Disconnected";
	}
    char *device_status = (char*)malloc(device_count * 12 + 1);

	for (int i = 0; i < device_count; i++) {
	    char temp[11];
	    sprintf(temp, "%d", device_list[i]);
	
	    for (int j = 0; j < 10; j++) {
	        device_status[i * 12 + j] = temp[j];
	    }
	
	    device_status[i * 12 + 10] = ',';
	    device_status[i * 12 + 11] = ' ';
	}

	device_status[device_count * 12 - 2] = '\0';

    snprintf(html_buffer, sizeof(html_buffer), html_template,
        (char *)sta_config.sta.ssid,
        (char *)sta_config.sta.password,
        (char *)ap_config.ap.ssid,
        (char *)ap_config.ap.password,
        sta_list.num,
        server_status,
        device_status
    );

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html_buffer, HTTPD_RESP_USE_STRLEN);
    free(device_status);
    return ESP_OK;
}

static esp_err_t submit_post_handler(httpd_req_t *req) {
    char buf[256];
    int ret, remaining = req->content_len;

    char ssid[32] = {0};
    char password[64] = {0};
    char device_ssid[32] = {0};
    char device_password[64] = {0};

    while (remaining > 0) {
        if ((ret = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf) - 1))) <= 0) {
            return ESP_FAIL;
        }
        remaining -= ret;
        buf[ret] = '\0';

        ESP_LOGI(TAG, "POST Body: %s", buf);

        char *ssid_ptr = strstr(buf, "SSID=");
        if (ssid_ptr) {
            sscanf(ssid_ptr + 5, "%31[^&]", ssid);
        }

        char *pass_ptr = strstr(buf, "Wifipass=");
        if (pass_ptr) {
            sscanf(pass_ptr + 9, "%63[^&]", password);
        }

        char *dev_ssid_ptr = strstr(buf, "Device_SSID=");
        if (dev_ssid_ptr) {
            sscanf(dev_ssid_ptr + 12, "%31[^&]", device_ssid);
        }

        char *dev_pass_ptr = strstr(buf, "Device_Wifipass=");
        if (dev_pass_ptr) {
            sscanf(dev_pass_ptr + 17, "%63[^&]", device_password);
        }
    }

    if (strlen(ssid) > 0 && strlen(password) > 0) {
        strncpy(Wifi_SSID, ssid, sizeof(Wifi_SSID));
        strncpy(Wifi_password, password, sizeof(Wifi_password));
    }

    if (strlen(device_ssid) > 0 && strlen(device_password) > 0) {
        strncpy(Device_SSID, device_ssid, sizeof(Device_SSID));
        strncpy(Device_password, device_password, sizeof(Device_password));
    }

	esp_wifi_disconnect();

	wifi_config_t wifi_sta_config = {
	    .sta = {
	        .threshold.authmode = WIFI_AUTH_WPA2_PSK,
	    },
	};
	strncpy((char *)wifi_sta_config.sta.ssid, Wifi_SSID, sizeof(wifi_sta_config.sta.ssid));
	strncpy((char *)wifi_sta_config.sta.password, Wifi_password, sizeof(wifi_sta_config.sta.password));
	
	esp_wifi_set_config(WIFI_IF_STA, &wifi_sta_config);
	esp_wifi_connect();
	
	wifi_config_t wifi_ap_config = {
	    .ap = {
	        .ssid_len = sizeof(wifi_ap_config.ap.password),
	        .max_connection = 4,
	        .authmode = WIFI_AUTH_WPA_WPA2_PSK
	    },
	};
	strncpy((char *)wifi_ap_config.ap.ssid, Device_SSID, sizeof(wifi_ap_config.ap.ssid));
	strncpy((char *)wifi_ap_config.ap.password, Device_password, sizeof(wifi_ap_config.ap.password));
	
	if (strlen((char *)wifi_ap_config.ap.password) == 0) {
	    wifi_ap_config.ap.authmode = WIFI_AUTH_OPEN;
	}
	
	esp_wifi_set_config(WIFI_IF_AP, &wifi_ap_config);
	
	ESP_LOGI(TAG, "Wi-Fi settings updated and reconnected.");
	
    save_basic_info();

    httpd_resp_send(req, "<h2>SSID and Password Updated!</h2><a href='/'>Go back</a>", HTTPD_RESP_USE_STRLEN);

    ESP_LOGI(TAG, "Updated SSID: %s, Password: %s", Wifi_SSID, Wifi_password);
    ESP_LOGI(TAG, "Updated Device SSID: %s, Password: %s", Device_SSID, Device_password);

    return ESP_OK;
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
    esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;
    switch (event_id) {
        case WEBSOCKET_EVENT_CONNECTED:
            ESP_LOGI(TAG, "WebSocket connected");
            break;
        case WEBSOCKET_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "WebSocket disconnected");
            break;
        case WEBSOCKET_EVENT_DATA:
            ESP_LOGI(TAG, "Received data: %.*s", data->data_len, (char *)data->data_ptr);

            char *rx_data = strndup((char *)data->data_ptr, data->data_len);
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
                                for(int j = 0;j < device_count;j++){
									if(device_info[j]->code == id->valueint){
										device_info[j]->danger=distance->valueint;
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
            ESP_LOGE(TAG, "WebSocket error occurred");
            break;
    }
}

void wss_client_task(void *pvParameters) {
    esp_websocket_client_config_t websocket_cfg = {
        .uri = "wss://" HOST_IP_ADDR ":" PORT"/",
    };

    while (1) {
		if (!esp_websocket_client_is_connected(client)) {
		    esp_websocket_client_stop(client);
		    esp_websocket_client_destroy(client);
		    client = NULL;
		}
        
        if (gps_data_all_receive) {
            char *server_send_string_JSON = make_json_payload();
            if (server_send_string_JSON != NULL) {
                if (esp_websocket_client_send_text(client, server_send_string_JSON, strlen(server_send_string_JSON), portMAX_DELAY) > 0) {
                    ESP_LOGI(TAG, "Sent JSON payload");
                } else {
                    ESP_LOGE(TAG, "Failed to send payload");
               	 	esp_websocket_client_stop(client);
                    esp_websocket_client_destroy(client);
                    client = NULL;
                }
                free(server_send_string_JSON);
            }
            gps_data_all_receive = false;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    esp_websocket_client_stop(client);
    esp_websocket_client_destroy(client);
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
			char gps_get_messege[20];
			
			sprintf(gps_get_messege,"%u,%d", device_info[i]->code,device_info[i]->danger);
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
    
    load_basic_info();
    load_array(device_list, MAX_DEVICE_CNT);
	
	gpio_num_t pins[] = {CTRL, FIND_HOST, Enter_AND_SPACE, UP,DOWN};
	for (int i = 0; i < sizeof(pins)/sizeof(pins[0]); i++) {
	    gpio_set_direction(pins[i], GPIO_MODE_INPUT);
	    gpio_set_pull_mode(pins[i], GPIO_FLOATING);
	}
	
	wifi_init_softap_sta();
	lora_init();
	start_webserver();
	
	xTaskCreatePinnedToCore(wss_client_task, "wss_client", 8192, NULL, 5, NULL, 1); 
	xTaskCreatePinnedToCore(lora_host_task, "LoRa_host", 4096, NULL, 5, NULL, 1);    
	xTaskCreatePinnedToCore(gps_task, "gps_task", 4096, NULL, 10, NULL, 1);          
    
    
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
			}
		}
    }
}