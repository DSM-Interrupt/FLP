/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "stdint.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct GpsData{
	float latitude;
	float longitude;
}GpsData;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TERMINAL_ID_INT 2041235739
#define TERMINAL_ID_CHAR "2041235739"
#define LORA_MAX_PACKET_SIZE 255
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t gps_rx_buffer[1];
char gps_sentence[128];
uint8_t gps_index = 0;
GpsData SaveGpsData;
int host_id;
static int danger = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void lora_write_register(uint8_t reg, uint8_t value) {
    uint8_t tx_buf[2] = { (reg | 0x80), value };

    HAL_GPIO_WritePin(GPIOA, SX1278_NSS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, tx_buf, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOA, SX1278_NSS_Pin, GPIO_PIN_SET);
}

uint8_t lora_read_register(uint8_t reg) {
    uint8_t tx_buf[2] = { (reg & 0x7F), 0x00 };
    uint8_t rx_buf[2] = { 0 };

    HAL_GPIO_WritePin(GPIOA, SX1278_NSS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOA, SX1278_NSS_Pin, GPIO_PIN_SET);

    return rx_buf[1];
}

void lora_init(void) {
    HAL_GPIO_WritePin(GPIOB, SX1278_RESET_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(GPIOB, SX1278_RESET_Pin, GPIO_PIN_SET);
    HAL_Delay(10);

    lora_write_register(0x01, 0x81);

    uint64_t frf = ((uint64_t)433000000 << 19) / 32000000;
    lora_write_register(0x06, (frf >> 16) & 0xFF);
    lora_write_register(0x07, (frf >> 8) & 0xFF);
    lora_write_register(0x08, (frf >> 0) & 0xFF);

    lora_write_register(0x1D, 0x72);
    lora_write_register(0x1E, 0xC4);
    lora_write_register(0x22, LORA_MAX_PACKET_SIZE);


    lora_write_register(0x12, 0xFF);
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

void parse_gps_sentence(char* gps_input) {
    if (!(strstr(gps_input, "$GPGGA") || strstr(gps_input, "$GPRMC"))) {
        return;
    }

    char *token;
    int field_index = 0;
    char lat[16] = {0}, lon[16] = {0};
    char lat_dir = 0, lon_dir = 0;

    token = strtok(gps_input, ",");

    while (token != NULL) {
        field_index++;

        if (field_index == 3) {
            strncpy(lat, token, sizeof(lat) - 1);
        }
        else if (field_index == 4) {
            lat_dir = token[0];
        }
        else if (field_index == 5) {
            strncpy(lon, token, sizeof(lon) - 1);
        }
        else if (field_index == 6) {
            lon_dir = token[0];
        }

        token = strtok(NULL, ",");
    }

    if (strlen(lat) > 0 && strlen(lon) > 0) {
        double conv_lat = convert_to_decimal_degrees(lat, lat_dir);
        double conv_lon = convert_to_decimal_degrees(lon, lon_dir);

        if (conv_lat >= -90 && conv_lat <= 90 && conv_lon >= -180 && conv_lon <= 180) {
            SaveGpsData.latitude = conv_lat;
            SaveGpsData.longitude = conv_lon;
        }
    }
}


void lora_send_packet(const char* data) {
    size_t len = strlen(data);
    if (len > 255) len = 255;

    lora_write_register(0x01, 0x81);
    lora_write_register(0x0E, 0x00);
    lora_write_register(0x0D, 0x00);

    for (size_t i = 0; i < len; i++) {
        lora_write_register(0x00, data[i]);
    }

    lora_write_register(0x22, len);
    lora_write_register(0x01, 0x83);
    while ((lora_read_register(0x12) & 0x08) == 0) {
        HAL_Delay(10);
    }
    lora_write_register(0x12, 0xFF);
}

int lora_receive_packet(char *buf, size_t max_len) {
    if (buf == NULL || max_len == 0) return 0;

    int len = 0;
    uint8_t irq_flags = lora_read_register(0x12);

    if (irq_flags & 0x40) {
        if (irq_flags & 0x20) {
            lora_write_register(0x12, 0xFF);
            return 0;
        }

        len = lora_read_register(0x13);

        if (len > max_len - 1){
        	len = max_len - 1;
        }

        uint8_t fifo_addr = lora_read_register(0x10);
        lora_write_register(0x0D, fifo_addr);

        for (int i = 0; i < len; i++) {
            buf[i] = lora_read_register(0x00);
        }
        buf[len] = '\0';

        int rssi = lora_read_register(0x1A);
        printf("LoRa RX %d bytes (RSSI=%d): %s\r\n", len, rssi, buf);
    }
    lora_write_register(0x12, 0xFF);
    return len;
}

void buzzer_beep_once() {
    HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, GPIO_PIN_SET);
    HAL_Delay(10000);
    HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, GPIO_PIN_RESET);
}

void buzzer_alarm_repeat() {
    for (int i = 0; i < 3; i++) {
        HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, GPIO_PIN_SET);
        HAL_Delay(2000);
        HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, GPIO_PIN_RESET);
        HAL_Delay(2000);
    }
}

void buzzer_off() {
    HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, GPIO_PIN_RESET);
}

void lora_receive_handler(void) {
    char buf[256] = {0};
    int packet_len = lora_receive_packet(buf,255);

    if (packet_len > 0) {
        buf[packet_len] = '\0';
        printf("LoRa Received: %s\n", buf);
        if (strstr(buf, "getgps") != NULL) {
        	char * token;
        	token = strtok(buf,",");
        	if(strstr(token,TERMINAL_ID_CHAR)){
        		token = strtok(buf,NULL);
        		host_id = atoi(token);

        		token = strtok(buf,NULL);
        		danger = atoi(token);

        		switch (danger) {
					case 0:
						buzzer_off();
						break;
					case 1:
						buzzer_beep_once();
						break;
					case 2:
					case 3:
						buzzer_alarm_repeat();
						break;
					default:
						buzzer_off();
						break;
				}

        		char response[64];
				snprintf(response, sizeof(response), "%u,%.4f,%.4f %u", TERMINAL_ID_INT, SaveGpsData.latitude, SaveGpsData.longitude, host_id);

				lora_send_packet(response);
				printf("LoRa Sent: %s\n", response);
        	}
        }
    }
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  lora_init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, gps_rx_buffer, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SX1278_NSS_Pin|buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ID_Pin|SX1278_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SX1278_NSS_Pin buzzer_Pin */
  GPIO_InitStruct.Pin = SX1278_NSS_Pin|buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO0_Pin */
  GPIO_InitStruct.Pin = DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : find_host_Pin */
  GPIO_InitStruct.Pin = find_host_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(find_host_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ID_Pin SX1278_RESET_Pin */
  GPIO_InitStruct.Pin = ID_Pin|SX1278_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        if (gps_rx_buffer[0] == '\n') {
            gps_sentence[gps_index] = '\0';
            gps_index = 0;
            parse_gps_sentence(gps_sentence);
        } else {
            gps_sentence[gps_index++] = gps_rx_buffer[0];
            if (gps_index >= 127) gps_index = 0;
        }
        HAL_UART_Receive_IT(&huart2, gps_rx_buffer, 1);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == DIO0_Pin) {
    	lora_receive_handler();
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
