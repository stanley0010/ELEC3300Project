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
#include "ili9341.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define min(a,b) (((a)<(b))?(a):(b))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_FSMC_Init(void);
/* USER CODE BEGIN PFP */
void demoLCD(int i);
unsigned long testFillScreen();
unsigned long testText();
unsigned long testLines(uint16_t color);
unsigned long testFastLines(uint16_t color1, uint16_t color2);
unsigned long testRects(uint16_t color);
unsigned long testFilledRects(uint16_t color1, uint16_t color2);
unsigned long testFilledCircles(uint8_t radius, uint16_t color);
unsigned long testCircles(uint8_t radius, uint16_t color);
unsigned long testTriangles();
unsigned long testFilledTriangles();
unsigned long testRoundRects();
unsigned long testFilledRoundRects();
unsigned long testDrawImage();
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_FSMC_Init();
  /* USER CODE BEGIN 2 */
	LCD_BL_ON();
	lcdInit();
	int i = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		demoLCD(i);
//		i++;
	}

}

void demoLCD(int i) {

	lcdSetOrientation(1);

	lcdSetTextFont(&Font16);
	lcdSetCursor(0, lcdGetHeight() - lcdGetTextFont()->Height - 1);
	lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
	lcdPrintf("Time: %4lu ms", testDrawImage());
	HAL_Delay(2000);

//	lcdSetOrientation(i % 4);
//
//	uint32_t t = testFillScreen();
//	lcdSetTextFont(&Font16);
//	lcdSetCursor(0, lcdGetHeight() - lcdGetTextFont()->Height - 1);
//	lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
//	lcdPrintf("Time: %4lu ms", t);
//	HAL_Delay(2000);
//
//	t = HAL_GetTick();
//	lcdTest();
//	lcdSetTextFont(&Font16);
//	lcdSetCursor(0, lcdGetHeight() - lcdGetTextFont()->Height - 1);
//	lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
//	lcdPrintf("Time: %4lu ms", HAL_GetTick() - t);
//	HAL_Delay(2000);
//
//	t = testText();
//	lcdSetTextFont(&Font16);
//	lcdSetCursor(0, lcdGetHeight() - lcdGetTextFont()->Height - 1);
//	lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
//	lcdPrintf("Time: %4lu ms", t);
//	HAL_Delay(2000);
//
//	lcdSetTextFont(&Font16);
//	lcdSetCursor(0, lcdGetHeight() - lcdGetTextFont()->Height - 1);
//	lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
//	lcdPrintf("Time: %4lu ms", testLines(COLOR_CYAN));
//	HAL_Delay(2000);
//
//	lcdSetTextFont(&Font16);
//	lcdSetCursor(0, lcdGetHeight() - lcdGetTextFont()->Height - 1);
//	lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
//	lcdPrintf("Time: %4lu ms", testFastLines(COLOR_RED, COLOR_BLUE));
//	HAL_Delay(2000);
//
//	lcdSetTextFont(&Font16);
//	lcdSetCursor(0, lcdGetHeight() - lcdGetTextFont()->Height - 1);
//	lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
//	lcdPrintf("Time: %4lu ms", testRects(COLOR_GREEN));
//	HAL_Delay(2000);
//
//	lcdSetTextFont(&Font16);
//	lcdSetCursor(0, lcdGetHeight() - lcdGetTextFont()->Height - 1);
//	lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
//	lcdPrintf("Time: %4lu ms", testFilledRects(COLOR_YELLOW, COLOR_MAGENTA));
//	HAL_Delay(2000);
//
//	lcdSetTextFont(&Font16);
//	lcdSetCursor(0, lcdGetHeight() - lcdGetTextFont()->Height - 1);
//	lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
//	lcdPrintf("Time: %4lu ms", testFilledCircles(10, COLOR_MAGENTA));
//	HAL_Delay(2000);
//
//	lcdSetTextFont(&Font16);
//	lcdSetCursor(0, lcdGetHeight() - lcdGetTextFont()->Height - 1);
//	lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
//	lcdPrintf("Time: %4lu ms", testCircles(10, COLOR_WHITE));
//	HAL_Delay(2000);
//
//	lcdSetTextFont(&Font16);
//	lcdSetCursor(0, lcdGetHeight() - lcdGetTextFont()->Height - 1);
//	lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
//	lcdPrintf("Time: %4lu ms", testTriangles());
//	HAL_Delay(2000);
//
//	lcdSetTextFont(&Font16);
//	lcdSetCursor(0, lcdGetHeight() - lcdGetTextFont()->Height - 1);
//	lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
//	lcdPrintf("Time: %4lu ms", testFilledTriangles());
//	HAL_Delay(2000);
//
//	lcdSetTextFont(&Font16);
//	lcdSetCursor(0, lcdGetHeight() - lcdGetTextFont()->Height - 1);
//	lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
//	lcdPrintf("Time: %4lu ms", testRoundRects());
//	HAL_Delay(2000);
//
//	lcdSetTextFont(&Font16);
//	lcdSetCursor(0, lcdGetHeight() - lcdGetTextFont()->Height - 1);
//	lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
//	lcdPrintf("Time: %4lu ms", testFilledRoundRects());
//	HAL_Delay(2000);
//
//	lcdSetTextFont(&Font16);
//	lcdSetCursor(0, lcdGetHeight() - lcdGetTextFont()->Height - 1);
//	lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
//	lcdPrintf("Time: %4lu ms", testDrawImage());
//	HAL_Delay(2000);
}

unsigned long testFillScreen() {
	unsigned long start = HAL_GetTick(), t = 0;
	lcdFillRGB(COLOR_BLACK);
	t += HAL_GetTick() - start;
	lcdSetCursor(0, 0);
	lcdSetTextFont(&Font24);
	lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
	lcdPrintf("BLACK");
	HAL_Delay(1000);
	start = HAL_GetTick();
	lcdFillRGB(COLOR_RED);
	t += HAL_GetTick() - start;
	lcdSetCursor(0, 0);
	lcdSetTextFont(&Font24);
	lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
	lcdPrintf("RED");
	HAL_Delay(1000);
	start = HAL_GetTick();
	lcdFillRGB(COLOR_GREEN);
	t += HAL_GetTick() - start;
	lcdSetCursor(0, 0);
	lcdSetTextFont(&Font24);
	lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
	lcdPrintf("GREEN");
	HAL_Delay(1000);
	start = HAL_GetTick();
	lcdFillRGB(COLOR_BLUE);
	t += HAL_GetTick() - start;
	lcdSetCursor(0, 0);
	lcdSetTextFont(&Font24);
	lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
	lcdPrintf("BLUE");
	HAL_Delay(1000);
	start = HAL_GetTick();
	lcdFillRGB(COLOR_BLACK);
	return t += HAL_GetTick() - start;
}

unsigned long testText() {
	lcdFillRGB(COLOR_BLACK);
	unsigned long start = HAL_GetTick();
	lcdSetCursor(0, 0);
	lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
	lcdSetTextFont(&Font8);
	lcdPrintf("Hello World!\r\n");
	lcdSetTextColor(COLOR_YELLOW, COLOR_BLACK);
	lcdSetTextFont(&Font12);
	lcdPrintf("%i\r\n", 1234567890);
	lcdSetTextColor(COLOR_RED, COLOR_BLACK);
	lcdSetTextFont(&Font16);
	lcdPrintf("%#X\r\n", 0xDEADBEEF);
	lcdPrintf("\r\n");
	lcdSetTextColor(COLOR_GREEN, COLOR_BLACK);
	lcdSetTextFont(&Font20);
	lcdPrintf("Groop\r\n");
	lcdSetTextFont(&Font12);
	lcdPrintf("I implore thee,\r\n");
	lcdSetTextFont(&Font12);
	lcdPrintf("my foonting turlingdromes.\r\n");
	lcdPrintf("And hooptiously drangle me\r\n");
	lcdPrintf("with crinkly bindlewurdles,\r\n");
	lcdPrintf("Or I will rend thee\r\n");
	lcdPrintf("in the gobberwarts\r\n");
	lcdPrintf("with my blurglecruncheon,\r\n");
	lcdPrintf("see if I don't!\r\n");
	return HAL_GetTick() - start;
}

unsigned long testLines(uint16_t color) {
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
	lcdFillRGB(COLOR_BLACK);

	x1 = 0;
	y1 = h - 1;
	y2 = 0;
	start = HAL_GetTick();

	for (x2 = 0; x2 < w; x2 += 6)
		lcdDrawLine(x1, y1, x2, y2, color);
	x2 = w - 1;
	for (y2 = 0; y2 < h; y2 += 6)
		lcdDrawLine(x1, y1, x2, y2, color);
	t += HAL_GetTick() - start;

	HAL_Delay(1000);
	lcdFillRGB(COLOR_BLACK);

	x1 = w - 1;
	y1 = h - 1;
	y2 = 0;

	start = HAL_GetTick();

	for (x2 = 0; x2 < w; x2 += 6)
		lcdDrawLine(x1, y1, x2, y2, color);
	x2 = 0;
	for (y2 = 0; y2 < h; y2 += 6)
		lcdDrawLine(x1, y1, x2, y2, color);

	return t += HAL_GetTick() - start;
}

unsigned long testFastLines(uint16_t color1, uint16_t color2) {
	unsigned long start;
	int x, y, w = lcdGetWidth(), h = lcdGetHeight();

	lcdFillRGB(COLOR_BLACK);
	start = HAL_GetTick();
	for (y = 0; y < h; y += 5)
		lcdDrawHLine(0, w, y, color1);
	for (x = 0; x < w; x += 5)
		lcdDrawVLine(x, 0, h, color2);

	return HAL_GetTick() - start;
}

unsigned long testRects(uint16_t color) {
	unsigned long start;
	int n, i, i2, cx = lcdGetWidth() / 2, cy = lcdGetHeight() / 2;

	lcdFillRGB(COLOR_BLACK);
	n = min(lcdGetWidth(), lcdGetHeight());
	start = HAL_GetTick();
	for (i = 2; i < n; i += 6) {
		i2 = i / 2;
		lcdDrawRect(cx - i2, cy - i2, i, i, color);
	}

	return HAL_GetTick() - start;
}

unsigned long testFilledRects(uint16_t color1, uint16_t color2) {
	unsigned long start, t = 0;
	int n, i, i2, cx = lcdGetWidth() / 2 - 1, cy = lcdGetHeight() / 2 - 1;

	lcdFillRGB(COLOR_BLACK);
	n = min(lcdGetWidth(), lcdGetHeight());

	for (i = n; i > 0; i -= 6) {
		i2 = i / 2;
		start = HAL_GetTick();
		lcdFillRect(cx - i2, cy - i2, i, i, color1);
		t += HAL_GetTick() - start;
		// Outlines are not included in timing results
		lcdDrawRect(cx - i2, cy - i2, i, i, color1);
	}

	return t;
}

unsigned long testFilledCircles(uint8_t radius, uint16_t color) {
	unsigned long start;
	int x, y, w = lcdGetWidth(), h = lcdGetHeight(), r2 = radius * 2;

	lcdFillRGB(COLOR_BLACK);
	start = HAL_GetTick();
	for (x = radius; x < w; x += r2) {
		for (y = radius; y < h; y += r2) {
			lcdFillCircle(x, y, radius, color);
		}
	}

	return HAL_GetTick() - start;
}

unsigned long testCircles(uint8_t radius, uint16_t color) {
	unsigned long start;
	int x, y, r2 = radius * 2, w = lcdGetWidth() + radius, h = lcdGetHeight()
			+ radius;

// Screen is not cleared for this one -- this is
// intentional and does not affect the reported time.
	start = HAL_GetTick();
	for (x = 0; x < w; x += r2) {
		for (y = 0; y < h; y += r2) {
			lcdDrawCircle(x, y, radius, color);
		}
	}

	return HAL_GetTick() - start;
}

unsigned long testTriangles() {
	unsigned long start;
	int n, i, cx = lcdGetWidth() / 2 - 1, cy = lcdGetHeight() / 2 - 1;

	lcdFillRGB(COLOR_BLACK);
	n = min(cx, cy);
	start = HAL_GetTick();
	for (i = 0; i < n; i += 5) {
		lcdDrawTriangle(cx, cy - i, // peak
		cx - i, cy + i, // bottom left
		cx + i, cy + i, // bottom right
		lcdColor565(i, i, i));
	}

	return HAL_GetTick() - start;
}

unsigned long testFilledTriangles() {
	unsigned long start, t = 0;
	int i, cx = lcdGetWidth() / 2 - 1, cy = lcdGetHeight() / 2 - 1;

	lcdFillRGB(COLOR_BLACK);
	for (i = min(cx, cy); i > 10; i -= 5) {
		start = HAL_GetTick();
		lcdFillTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
				lcdColor565(0, i * 10, i * 10));
		t += HAL_GetTick() - start;
		lcdFillTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
				lcdColor565(i * 10, i * 10, 0));
	}

	return t;
}

unsigned long testRoundRects() {
	unsigned long start;
	int w, i, i2, cx = lcdGetWidth() / 2 - 1, cy = lcdGetHeight() / 2 - 1;

	lcdFillRGB(COLOR_BLACK);
	w = lcdGetWidth(), lcdGetHeight();
	start = HAL_GetTick();
	for (i = 0; i < w; i += 6) {
		i2 = i / 2;
		lcdDrawRoundRect(cx - i2, cy - i2, i, i, i / 8, lcdColor565(i, 0, 0));
	}

	return HAL_GetTick() - start;
}

unsigned long testFilledRoundRects() {
	unsigned long start;
	int i, i2, cx = lcdGetWidth() / 2 - 1, cy = lcdGetHeight() / 2 - 1;

	lcdFillRGB(COLOR_BLACK);
	start = HAL_GetTick();
	for (i = min(lcdGetWidth(), lcdGetHeight()); i > 20; i -= 6) {
		i2 = i / 2;
		lcdFillRoundRect(cx - i2, cy - i2, i, i, i / 8, lcdColor565(0, i, 0));
	}

	return HAL_GetTick() - start;
}

unsigned long testDrawImage() {
	unsigned long start;

	lcdFillRGB(COLOR_BLACK);
	start = HAL_GetTick();
	if (lcdGetOrientation() == LCD_ORIENTATION_LANDSCAPE
			|| lcdGetOrientation() == LCD_ORIENTATION_LANDSCAPE_MIRROR) {
		lcdDrawImage((lcdGetWidth() - bmSTLogo.xSize) / 2, 0, &bmSTLogo);
//		lcdDrawImage(0, 0, &bmSTLogo);
	} else {
		lcdDrawImage(0, (lcdGetHeight() - bmSTLogo.ySize) / 2, &bmSTLogo);
	}
	return HAL_GetTick() - start;
}



/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}
	/* USER CODE END 3 */
/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

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
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, DOUT_Pin|IRQ_Pin|DCLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);

  /*Configure GPIO pin : DIN_Pin */
  GPIO_InitStruct.Pin = DIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DOUT_Pin IRQ_Pin DCLK_Pin PE1 */
  GPIO_InitStruct.Pin = DOUT_Pin|IRQ_Pin|DCLK_Pin|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

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

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

  /* USER CODE BEGIN FSMC_Init 0 */

  /* USER CODE END FSMC_Init 0 */

  FSMC_NORSRAM_TimingTypeDef Timing = {0};

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

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
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
void Error_Handler(void)
{
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