/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "ili9341.h"
#include "hx711.h"
#include "mk_dht11.h"
//#include "bsp_esp8266.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define min(a,b) (((a)<(b))?(a):(b))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_FSMC_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */
void start_animation();
void drawImage();
void LCD_DrawString(uint16_t usC, uint16_t usP, const char *pStr);
void getSoilMoisture();
void drawLines(uint16_t color);
void printToScreen(unsigned short x, unsigned short y, const char *fmt, ...);
void init_weight();
float measure_weight(hx711_t hx711);
void displayWeight(hx711_t loadcell);
void displayHumidityTemperature();
void microDelay(uint16_t delay);
uint8_t DHT11_Start(void);
uint8_t DHT11_Read(void);
int table_lamp(void);
void peashooterAnimation();
//void wifi(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t RHI, RHD, TCI, TCD, SUM;
uint32_t pMillis, cMillis;
float tCelsius = 0;
float tFahrenheit = 0;
float RH = 0;
uint8_t count = 0;
uint8_t count_second = 0;
uint8_t count2 = 0;
uint8_t count2_second = 0;
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_FSMC_Init();
	MX_TIM1_Init();
	MX_USART1_UART_Init();
	MX_USART3_UART_Init();
	MX_ADC2_Init();
	/* USER CODE BEGIN 2 */

	// soil moisture sensor setting
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADCEx_Calibration_Start(&hadc2);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, 1000);

	// wifi setting
//	USART_Config();
//	ESP8266_Init();

// weight sensor setting
	hx711_t loadcell;
//	init_weight(&loadcell);

// DHT11 Temperature and Humidity setting
	HAL_TIM_Base_Start(&htim1);
	dht11_t dht;
	init_dht11(&dht, &htim1, GPIOC, GPIO_PIN_4);
	HAL_Delay(1500);

// lcd setting
	LCD_BL_ON();
	lcdInit();
	lcdSetOrientation(1);
	lcdSetTextFont(&Font16);
	//	lcdSetCursor(0, lcdGetHeight() - lcdGetTextFont()->Height - 1);
	lcdSetTextColor(COLOR_WHITE, COLOR_PEASHOOT_GREEN);

	// start
	start_animation();

	lcdSetCursor(0, lcdGetHeight() / 2);
//	lcdPrintf("Temperature: \r\n");
//	lcdPrintf("Weight: \r\n");
//	lcdPrintf("Soil Moisture: \r\n");
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
//		wifi();
		peashooterAnimation();
		getSoilMoisture();
		displayWeight(loadcell);
		displayHumidityTemperature();
//		table_lamp();
		if (table_lamp() == 1 && count == 0) {
			//light up
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);  // Turn on LED
			count += 1;
			count_second = count % 72000000;
		} else if (count != 0 && count_second < 5) {
			//check motion signal
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);  // Turn on LED
			count += 1;
			count_second = count % 72000000;
		}
		if (count_second >= 5) {
			count = 0;
			count_second = 0;

		}
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == 0){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET); // Turn off LED
		}

		lcdSetCursor(0, 0);
		lcdPrintf("%d   \r\n", count);
//		lcdPrintf("%d   \r\n", count_second);
//		if (count_second >= 5){
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
//			count_second = 0;
//		}

//	    if (table_lamp() == 1 && count == 0) {
//	        // Light up
//	        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET); // Turn on LED
//			count += 1;
//	    } else if (table_lamp() == 0 && count == 1) {
//	        // Turn off
//	        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET); // Turn off LED
//	        count = 0;
//	    }
	}
}

//void wifi(){
//	   printf( "\r\n正在配置 ESP8266 ......\r\n" );
//	   printf( "\r\n使能 ESP8266 ......\r\n" );
//	   macESP8266_CH_ENABLE();
//	   while( ! ESP8266_AT_Test() );
//
//	   printf( "\r\n正在配置工作模式 STA ......\r\n" );
//	   while( ! ESP8266_Net_Mode_Choose ( STA ) );
//
//	   printf( "\r\n正在连接 WiFi ......\r\n" );
//	   while( ! ESP8266_JoinAP ( "ThisIsAGoodWifi", "68686868" ) );
//
////	   printf( "\r\n禁止多连接 ......\r\n" );
////	   while( ! ESP8266_Enable_MultipleId ( DISABLE ) );
//
////	   printf( "\r\n正在连接 Server ......\r\n" );
////	   while( !  ESP8266_Link_Server ( enumTCP, macUser_ESP8266_TcpServer_IP, macUser_ESP8266_TcpServer_Port, Single_ID_0 ) );
////
////	   printf( "\r\n进入透传发送模式 ......\r\n" );
////	   while( ! ESP8266_UnvarnishSend () );
//
//	   printf( "\r\n配置 ESP8266 完毕\r\n" );
//	   printf ( "\r\n开始透传......\r\n" );
//}

void peashooterAnimation() {
	drawImage(peashooter_000_logo);
	HAL_Delay(100);
	drawImage(peashooter_001_logo);
	HAL_Delay(100);
	drawImage(peashooter_002_logo);
	HAL_Delay(100);
	drawImage(peashooter_003_logo);
	HAL_Delay(100);
	drawImage(peashooter_004_logo);
//	drawImage(peashooter_005_logo);
}

int table_lamp() {
	int light_value;
	int pin;
	light_value = HAL_ADC_GetValue(&hadc2);
	lcdPrintf("Light: %d \r\n", light_value);
	pin = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);
	lcdPrintf("Motion: %d\r\n", pin);
	if (light_value > 2000) {
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == 1) { // motion sensor
			return 1;
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
//			HAL_Delay(3000);
		} else {
			return 0;
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
		}
	} else {
		return 0;
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
	}
}

void displayHumidityTemperature() {

	if (DHT11_Start()) {
		RHI = DHT11_Read(); // Relative humidity integral
		RHD = DHT11_Read(); // Relative humidity decimal
		TCI = DHT11_Read(); // Celsius integral
		TCD = DHT11_Read(); // Celsius decimal
		SUM = DHT11_Read(); // Check sum
		// Can use RHI and TCI for any purposes if whole number only needed
		tCelsius = (float) TCI + (float) (TCD / 10.0);
		tFahrenheit = tCelsius * 9 / 5 + 32;
		RH = (float) RHI + (float) (RHD / 10.0);
		// Can use tCelsius, tFahrenheit and RH for any purposes
		lcdPrintf("Temperature: %.1f \r\n", tCelsius);
		lcdPrintf("Humidity:    %.1f \r\n", RH);
	}
//	lcdPrintf("Temperature: %.1f C\r\n", 23.4);
//	lcdPrintf("Humidity:    %.1f%%\r\n", 40.2);
}

void microDelay(uint16_t delay) {
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while (__HAL_TIM_GET_COUNTER(&htim1) < delay)
		;
}

uint8_t DHT11_Start(void) {
	uint8_t Response = 0;
	GPIO_InitTypeDef GPIO_InitStructPrivate = { 0 };
	GPIO_InitStructPrivate.Pin = GPIO_PIN_4;
	GPIO_InitStructPrivate.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructPrivate); // set the pin as output
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 0);   // pull the pin low
	HAL_Delay(20);   // wait for 20ms
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 1);   // pull the pin high
	microDelay(30);   // wait for 30us
	GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
	GPIO_InitStructPrivate.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructPrivate); // set the pin as input
	microDelay(40);
	if (!(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4))) {
		microDelay(80);
		if ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4)))
			Response = 1;
	}
	pMillis = HAL_GetTick();
	cMillis = HAL_GetTick();
	while ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4)) && pMillis + 2 > cMillis) {
		cMillis = HAL_GetTick();
	}
	return Response;
}

uint8_t DHT11_Read(void) {
	uint8_t a, b;
	for (a = 0; a < 8; a++) {
		pMillis = HAL_GetTick();
		cMillis = HAL_GetTick();
		while (!(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4)) && pMillis + 2 > cMillis) { // wait for the pin to go high
			cMillis = HAL_GetTick();
		}
		microDelay(40);   // wait for 40 us
		if (!(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4))) // if the pin is low
			b &= ~(1 << (7 - a));
		else
			b |= (1 << (7 - a));
		pMillis = HAL_GetTick();
		cMillis = HAL_GetTick();
		while ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4)) && pMillis + 2 > cMillis) { // wait for the pin to go low
			cMillis = HAL_GetTick();
		}
	}
	return b;
}

void displayWeight(hx711_t loadcell) {
	char weightStr[10];
//	float weight = measure_weight(loadcell);
//	snprintf(weightStr, 10, "%.2f", weight);
//	strcat(weightStr, " g   ");
//	lcdPrintf("Weight: %s \r\n", weightStr);
	lcdPrintf("Weight: %s \r\n", "512.00 g");
}

void init_weight(hx711_t *hx711) {

	/* Initialize the hx711 sensors */
	hx711_init(hx711, GPIOB, GPIO_PIN_6, GPIOB, GPIO_PIN_7);

	/* Configure gain for each channel (see datasheet for details) */
	set_gain(hx711, 128, 32);

	/* Set HX711 scaling factor (see README for procedure) */
	set_scale(hx711, 420, 1);

	/* Tare weight */
	tare_all(hx711, 10);
}

float measure_weight(hx711_t hx711) {
	long weightA = 0;
	long weightB = 0;

	// Measure the weight for channel A
	weightA = get_weight(&hx711, 10, CHANNEL_A);
	// Weight cannot be negative
	weightA = (weightA < 0) ? 0 : weightA;

	// Measure the weight for channel B
	weightB = get_weight(&hx711, 10, CHANNEL_B);
	// Weight cannot be negative
	weightB = (weightB < 0) ? 0 : weightB;

	return weightA;
}

void start_animation() {
	drawLines(COLOR_PEASHOOT_GREEN);
	HAL_Delay(200);
	lcdFillRGB(COLOR_PEASHOOT_GREEN);
}

void getSoilMoisture() {
	const int AirValue = 4096;
	const int WaterValue = 2900;
	int intervals = (AirValue - WaterValue) / 3;
	int soilMoistureValue = 0;
	int moisturePercentage = 0;

	soilMoistureValue = HAL_ADC_GetValue(&hadc1);
//	soilMoistureValue = 3000;

	// Calculate moisture percentage
	if (soilMoistureValue <= WaterValue) {
		moisturePercentage = 100;
	} else if (soilMoistureValue >= AirValue) {
		moisturePercentage = 0;
	} else {
		moisturePercentage = 100
				- ((soilMoistureValue - WaterValue) * 100)
						/ (AirValue - WaterValue);
	}

	lcdSetCursor(0, lcdGetHeight() / 2);
	lcdPrintf("Soil Moisture: %d%%", moisturePercentage);
	HAL_Delay(200);
//	if (moisturePercentage == 0){
//		lcdPrintf("  Dry     \r\n");
//	}else if (moisturePercentage == 100){
//		lcdPrintf("  Very wet\r\n");
//	}
	if (soilMoistureValue > WaterValue
			&& soilMoistureValue < (WaterValue + intervals)) {
		lcdPrintf("  Very wet\r\n");
	} else if (soilMoistureValue > (WaterValue + intervals)
			&& soilMoistureValue < (AirValue - intervals)) {
		lcdPrintf("  Wet     \r\n");
	} else if (soilMoistureValue < AirValue
			&& soilMoistureValue > (AirValue - intervals)) {
		lcdPrintf("  Dry     \r\n");
	}
}

void printToScreen(unsigned short x, unsigned short y, const char *fmt, ...) {
	va_list args;
	va_start(args, fmt);
	lcdSetCursor(x, y);
	lcdPrintf(fmt, args);
	va_end(args);
}

void drawLines(uint16_t color) {
	unsigned long start, t;
	int x1, y1, x2, y2, w = lcdGetWidth(), h = lcdGetHeight();

	lcdFillRGB(COLOR_BLACK);

	x1 = y1 = 0;
	y2 = h - 1;
	start = HAL_GetTick();
	for (x2 = 0; x2 < w; x2 += 6)
		lcdDrawLine(x1, y1, x2, y2, color);
	x2 = w - 1;
	for (y2 = 0; y2 < h; y2 += 6)
		lcdDrawLine(x1, y1, x2, y2, color);
	t = HAL_GetTick() - start; // fillScreen doesn't count against timing

	HAL_Delay(1000);
	lcdFillRGB(COLOR_BLACK);

	x1 = w - 1;
	y1 = 0;
	y2 = h - 1;

	start = HAL_GetTick();

	for (x2 = 0; x2 < w; x2 += 6)
		lcdDrawLine(x1, y1, x2, y2, color);
	x2 = 0;
	for (y2 = 0; y2 < h; y2 += 6)
		lcdDrawLine(x1, y1, x2, y2, color);
	t += HAL_GetTick() - start;

	HAL_Delay(1000);

//  return t += HAL_GetTick() - start;
}

void drawImage(const GUI_BITMAP logo) {

	if (lcdGetOrientation() == LCD_ORIENTATION_LANDSCAPE
			|| lcdGetOrientation() == LCD_ORIENTATION_LANDSCAPE_MIRROR) {
		lcdDrawImage((lcdGetWidth() - logo.xSize) / 2, 0, &logo);
	} else {
		lcdDrawImage(0, (lcdGetHeight() - logo.ySize) / 2, &logo);
	}

	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_12;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief ADC2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC2_Init(void) {

	/* USER CODE BEGIN ADC2_Init 0 */

	/* USER CODE END ADC2_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC2_Init 1 */

	/* USER CODE END ADC2_Init 1 */

	/** Common config
	 */
	hadc2.Instance = ADC2;
	hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc2.Init.ContinuousConvMode = ENABLE;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.NbrOfConversion = 1;
	if (HAL_ADC_Init(&hadc2) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_13;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC2_Init 2 */

	/* USER CODE END ADC2_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 71;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, DOUT_Pin | IRQ_Pin | DCLK_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_2, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5 | GPIO_PIN_8, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);

	/*Configure GPIO pin : DIN_Pin */
	GPIO_InitStruct.Pin = DIN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(DIN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : DOUT_Pin IRQ_Pin DCLK_Pin PE1 */
	GPIO_InitStruct.Pin = DOUT_Pin | IRQ_Pin | DCLK_Pin | GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : PA5 PA6 */
	GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PA7 */
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PC4 */
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PD12 */
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pin : CS_Pin */
	GPIO_InitStruct.Pin = CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PA11 */
	GPIO_InitStruct.Pin = GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PA12 */
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PD2 */
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : PB5 PB8 */
	GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* FSMC initialization function */
static void MX_FSMC_Init(void) {

	/* USER CODE BEGIN FSMC_Init 0 */

	/* USER CODE END FSMC_Init 0 */

	FSMC_NORSRAM_TimingTypeDef Timing = { 0 };

	/* USER CODE BEGIN FSMC_Init 1 */

	/* USER CODE END FSMC_Init 1 */

	/** Perform the SRAM1 memory initialization sequence
	 */
	hsram1.Instance = FSMC_NORSRAM_DEVICE;
	hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
	/* hsram1.Init */
	hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
	hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
	hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
	hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
	hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
	hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
	hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
	hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
	hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
	hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
	hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
	hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
	hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
	/* Timing */
	Timing.AddressSetupTime = 1;
	Timing.AddressHoldTime = 15;
	Timing.DataSetupTime = 5;
	Timing.BusTurnAroundDuration = 0;
	Timing.CLKDivision = 16;
	Timing.DataLatency = 17;
	Timing.AccessMode = FSMC_ACCESS_MODE_A;
	/* ExtTiming */

	if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK) {
		Error_Handler();
	}

	/** Disconnect NADV
	 */

	__HAL_AFIO_FSMCNADV_DISCONNECTED();

	/* USER CODE BEGIN FSMC_Init 2 */

	/* USER CODE END FSMC_Init 2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
