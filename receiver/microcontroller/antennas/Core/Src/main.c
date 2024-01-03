/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#define ARM_MATH_CM4
#include "arm_math.h"
#include "arm_const_structs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * C++ version 0.4 char* style "itoa":
 * Written by Lukás Chmela
 * Released under GPLv3.
*/

char* itoa(int value, char* result, int base) {
	// check that the base if valid
	if (base < 2 || base > 36) { *result = '\0'; return result; }

	char* ptr = result, *ptr1 = result, tmp_char;
	int tmp_value;

	do {
		tmp_value = value;
		value /= base;
		*ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
	} while ( value );

	// Apply negative sign
	if (tmp_value < 0) *ptr++ = '-';
	*ptr-- = '\0';
	while(ptr1 < ptr) {
		tmp_char = *ptr;
		*ptr--= *ptr1;
		*ptr1++ = tmp_char;
	}
	return result;
}


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
  MX_USART2_UART_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  const int windowsize = 512;
  const int tolerance = 15000;
  const int samplesize = 12;
  const int samplerate = 2400000;

  const int targetfrequency0 = 800000;
  const int targetfrequency1 = 500000;

  const int amplification = 8;

  const int buffersize = windowsize << 1;
  const int nyquistsize = windowsize >> 1;

  int16_t buffer [buffersize] __attribute__ ((aligned (32)));
  int16_t fft [windowsize * 2];
  uint16_t mag [windowsize / 2];
  uint16_t ang [windowsize / 2];

  extern uint8_t dma_complete;
  dma_complete = 0;

  /*void TransferComplete(){
	  dma_available = 1;
	  HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_SET);
    }*/

  /*void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc){
	  dma_available = 1;
	  HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_SET);
  }*/
  //HAL_DMA_RegisterCallback(&hdma_adc1, HAL_DMA_XFER_HALFCPLT_CB_ID, TransferComplete);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*) buffer, buffersize);

  uint8_t dma_half_select = 0;
  while (1)
  {
	  int start;
	  int end;
	  if (dma_complete){
		  if (!dma_half_select){
			  //HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_RESET);
			  start = 0;
			  end = buffersize / 2;
		  }
		  else{

			  //HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_SET);
			  start = buffersize / 2;
			  end = buffersize;
		  }

		  dma_half_select = !dma_half_select;

		  int16_t dc_offset;
		  arm_mean_q15(buffer + start, windowsize, &dc_offset);

		  for (int i = start; i < end; i++){
			  int32_t val = ((buffer[i] - dc_offset) << (16 - samplesize)) * amplification;
			  if (val < -(1 << 15)){
				  val = -(1 << 15) + 1;
			  }
			  if (val >= (1 << 15)){
				  val = (1 << 15) - 1;
			  }
			  buffer[i] = val;
		  }

		  uint32_t doInverse = 0;
		  uint32_t doBitReverse = 1;
		  arm_rfft_instance_q15 rfft;
		  arm_rfft_init_q15(&rfft, windowsize, doInverse, doBitReverse);
		  arm_rfft_q15(&rfft, buffer + start, fft);
		  arm_cmplx_mag_q15(fft, mag, windowsize / 2);

		  /*int16_t *dirs = {1, 0, -1, 0, 0, 1, 0, -1};

		  for (int i = 0; i < windowsize / 2; i++){
			  int64_t res;
			  int maxidx = 0;
			  int max = -(1 << 64) + 1;
			  for (int i2 = 0; i2 < 4; i2++){
				  arm_dot_prod_q15(fft + i * 2, dirs + i2 * 2, 2, &res);
				  if (res > max){
					  max = res;
					  maxidx = i2;
				  }
			  }

			  ang[i] = maxidx;
		  }

		  uint8_t rx[1];
		  HAL_UART_Receive (&huart2, rx, 1, HAL_MAX_DELAY);

		  HAL_UART_Transmit(&huart2, (uint8_t*) "\r\n", 3, HAL_MAX_DELAY);
		  for (int i = 100; i < 115; i++){
			  char string[7];
			  itoa((int) ang[i], (char*) string, 10);

			  int pos;
			  for (pos = 0; pos < 6 && string[pos] != '\0'; pos++);
			  string[pos] = '\r';
			  string[pos + 1] = '\n';

			  HAL_UART_Transmit(&huart2, (uint8_t*) string, pos + 2, HAL_MAX_DELAY);
		  }*/

		  const int targetindex0 = (targetfrequency0 * windowsize + 0.5) / samplerate;
		  const int targetindex1 = (targetfrequency1 * windowsize + 0.5) / samplerate;
		  const int side = tolerance * windowsize / samplerate;
		  uint8_t recep = 0;
		  uint8_t data = 0;

		  uint16_t threshold = 0;
		  //arm_mean_q15(mag, windowsize / 2, &threshold);


		  for (int i = targetindex0 - side; i <= targetindex0 + side; i++){
			  if (i < nyquistsize){
				  if (mag[i] > threshold){
					  data = 0;
					  recep = 1;
					  break;
				  }
			  }
		  }
		  for (int i = targetindex1 - side; i <= targetindex1 + side; i++){
			  if (i < nyquistsize){
				  if (mag[i] > threshold){
					  recep = 1;
					  data = 1;
					  break;
				  }
			  }
		  }

		  if (recep){
			  HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_SET);
			  if(data){
				  HAL_UART_Transmit(&huart2, (uint8_t*) "1", 1, HAL_MAX_DELAY);
			  }
			  else{
				  HAL_UART_Transmit(&huart2, (uint8_t*) "0", 1, HAL_MAX_DELAY);
			  }
		  }
		  else{
			  HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_RESET);
			  HAL_UART_Transmit(&huart2, (uint8_t*) "2", 1, HAL_MAX_DELAY);
		  }


		  if (!dma_half_select){
			  dma_complete = 0;
			  HAL_ADC_Start_DMA(&hadc1, (uint32_t*) buffer, buffersize);
		  }
	  }


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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  huart2.Init.BaudRate = 115200;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
