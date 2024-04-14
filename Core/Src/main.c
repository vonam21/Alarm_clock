/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t tick[100];
uint8_t data_dht11[4] = {0x00,0x00,0x00,0x00};
volatile uint32_t tick_lan_truoc = 0;
volatile uint32_t tick_hien_tai =0;
volatile uint32_t thoi_gian_tick = 0;
int count_tick =0;


uint32_t debounceDelay = 200; // Th�?i gian ch�? debounce (miliseconds)
volatile uint32_t lastDebounceTime = 0; // Th�?i gian debounce cuối cùng
uint32_t currentTime =0;
volatile uint32_t lastDebounceTime2 = 0; // Th�?i gian debounce cuối cùng
uint32_t currentTime2 =0;
volatile uint32_t lastDebounceTime3 = 0; // Th�?i gian debounce cuối cùng
uint32_t currentTime3 =0;
volatile uint32_t lastDebounceTime4 = 0; // Th�?i gian debounce cuối cùng
uint32_t currentTime4 =0;
int flag_number=0;

int num1 =0;
int num2=0;
int num3=0;
int num4=0;
int num5 =0;
int num6 =0;
int num7=0;
int num8 =0;
int num9 =0;
int num0 =0;
int num_sao =0;
int num_thang =0;
volatile int led =0, quat =0, bom =0;
int mode = 0;
uint64_t count =0;
int flag_ngat_timer3 =1;
char  nhiet_do[20],do_am[20];
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Stop_IT(&htim2);
  HAL_TIM_Base_Stop_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // lưu ý khi cách đọc dht11 là dùng ngắt ngoài nên sử dụng ngắt ưu tiên cao nhất
	  // trước khi đọc dht11 thì gọi hàm doc_dht11() sau đó delay 10ms và gọi hàm xy_ly_tick_dht11
		doc_dht11();
		HAL_Delay(10);
		xu_ly_tick_dht11(tick,data_dht11); // hàm này lấy pointer cho ra data_dht11 là 1 mảng với
		/* tương ứng 	data_dht11[0] là phần nguyên độ ẩm
						data_dht11[0] là phần thập phân độ ẩm
						data_dht11[0] là phần nguyên nhiệt độ
						data_dht11[0] là phần thập phân nhiệt độ
						*/

		sprintf(nhiet_do,"Nhiet do: %d.%doC", data_dht11[2], data_dht11[3]); // hàm này ghép chuỗi thành chuỗi nhiet_do để tiện hiển thị LCD
		sprintf(do_am, "Do am:  %d.%d ", data_dht11[0],data_dht11[1]);		// hàm ghép chuỗi độ ẩm để hiển thị LCD
		HAL_Delay(5000);
		// mỗi khi nhấn nút nhấn thì sẽ tăng các giá trị num1,num2,num3,num4,.... lên 1
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 63;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 63;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_10|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA2 PA3 PA4
                           PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB10 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_10|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 11, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 11, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 11, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 11, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_2)
	{
		currentTime = HAL_GetTick(); // Lấy th�?i gian hiện tại
		if ((currentTime - lastDebounceTime) > debounceDelay)
		{
			// Cập nhật trạng thái nút nhấn chỉ khi đã qua th�?i gian debounce
			if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == 0)
			{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8,0);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9,1);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,1);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,1);
				if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == 0)
				{
					num1++;
					flag_number=1;
					HAL_TIM_Base_Start_IT(&htim3);
				} else {
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8,1);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9,0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,1);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,1);
					if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == 0)
					{
						num2++;
						flag_number=2;
						HAL_TIM_Base_Start_IT(&htim3);
					} else {
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8,1);
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9,1);
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,0);
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,1);
						if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == 0)
						{
							num3++;
							flag_number=3;
							HAL_TIM_Base_Start_IT(&htim3);
						} else {
							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8,1);
							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9,1);
							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,1);
							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,0);
							if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == 0)
							{

									led++;
									flag_number=10;
									HAL_TIM_Base_Start_IT(&htim3);

							}
						}
					}
				}

				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8,0);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9,0);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,0);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,0);
			}

			lastDebounceTime = currentTime; // Cập nhật th�?i gian debounce cuối cùng
		}
	}


	if(GPIO_Pin == GPIO_PIN_3)
		{
			currentTime = HAL_GetTick(); // Lấy th�?i gian hiện tại
			if ((currentTime - lastDebounceTime) > debounceDelay)
			{
				// Cập nhật trạng thái nút nhấn chỉ khi đã qua th�?i gian debounce
				if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == 0)
				{
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8,0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9,1);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,1);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,1);
					if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == 0)
					{
						num4++;
						flag_number=4;
						HAL_TIM_Base_Start_IT(&htim3);
					} else {
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8,1);
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9,0);
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,1);
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,1);
						if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == 0)
						{
							num5++;
							flag_number=5;
							HAL_TIM_Base_Start_IT(&htim3);
						} else {
							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8,1);
							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9,1);
							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,0);
							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,1);
							if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == 0)
							{
								num6++;
								flag_number=6;
								HAL_TIM_Base_Start_IT(&htim3);
							} else {
								HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8,1);
								HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9,1);
								HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,1);
								HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,0);
								if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == 0)
								{
										quat++;
										flag_number=10;
										HAL_TIM_Base_Start_IT(&htim3);

								}
							}
						}
					}

					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8,0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9,0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,0);
				}

				lastDebounceTime = currentTime; // Cập nhật th�?i gian debounce cuối cùng
			}
		}


	if(GPIO_Pin == GPIO_PIN_4)
		{
			currentTime = HAL_GetTick(); // Lấy th�?i gian hiện tại
			if ((currentTime - lastDebounceTime) > debounceDelay)
			{
				// Cập nhật trạng thái nút nhấn chỉ khi đã qua th�?i gian debounce
				if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == 0)
				{
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8,0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9,1);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,1);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,1);
					if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == 0)
					{
						num7++;
						flag_number=7;
						HAL_TIM_Base_Start_IT(&htim3);
					} else {
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8,1);
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9,0);
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,1);
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,1);
						if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == 0)
						{
							num8++;
							flag_number=8;
							HAL_TIM_Base_Start_IT(&htim3);
						} else {
							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8,1);
							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9,1);
							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,0);
							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,1);
							if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == 0)
							{
								num9++;
								flag_number=9;
								HAL_TIM_Base_Start_IT(&htim3);
							} else {
								HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8,1);
								HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9,1);
								HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,1);
								HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,0);
								if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == 0)
								{
										bom++;
										flag_number=10;
										HAL_TIM_Base_Start_IT(&htim3);
								}
							}
						}
					}

					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8,0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9,0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,0);
				}

				lastDebounceTime = currentTime; // Cập nhật th�?i gian debounce cuối cùng
			}
		}



	if(GPIO_Pin == GPIO_PIN_5)
		{
			currentTime = HAL_GetTick(); // Lấy th�?i gian hiện tại
			if ((currentTime - lastDebounceTime) > debounceDelay)
			{
				// Cập nhật trạng thái nút nhấn chỉ khi đã qua th�?i gian debounce
				if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == 0)
				{
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8,0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9,1);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,1);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,1);
					if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == 0)
					{
							num_sao++;
							flag_number=10;
							HAL_TIM_Base_Start_IT(&htim3);
					} else {
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8,1);
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9,0);
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,1);
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,1);
						if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == 0)
						{
							num0++;
							flag_number=0;
							HAL_TIM_Base_Start_IT(&htim3);
						} else {
							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8,1);
							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9,1);
							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,0);
							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,1);
							if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == 0)
							{
								num_thang++;
								flag_number=11;
								HAL_TIM_Base_Start_IT(&htim3);
							} else {
								HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8,1);
								HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9,1);
								HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,1);
								HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,0);
								if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == 0)
								{
										mode++;
										flag_number=10;
										HAL_TIM_Base_Start_IT(&htim3);
								}
							}
						}
					}

					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8,0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9,0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,0);
				}

				lastDebounceTime = currentTime; // Cập nhật th�?i gian debounce cuối cùng
			}
		}
	if(GPIO_Pin == GPIO_PIN_1)
		{

				tick_hien_tai = __HAL_TIM_GET_COUNTER(&htim2);
				thoi_gian_tick = tick_hien_tai - tick_lan_truoc;
				tick_lan_truoc = tick_hien_tai;
				tick[count_tick] = thoi_gian_tick;
				count_tick++;
				if(count_tick >84)
				{
					  HAL_TIM_Base_Stop_IT(&htim2);
					  HAL_NVIC_DisableIRQ(EXTI1_IRQn);
					  GPIO_InitTypeDef GPIO_InitStruct = {0};
					  GPIO_InitStruct.Pull = GPIO_NOPULL;
					  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP  ;
					  GPIO_InitStruct.Pin = GPIO_PIN_1;
					  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
					  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
				}
		}

}
void doc_dht11(void)
{
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_NVIC_DisableIRQ(EXTI1_IRQn);
	count_tick =0;
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT ;
	  GPIO_InitStruct.Pin = GPIO_PIN_1;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  HAL_Delay(1);

	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP  ;
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);

	HAL_Delay(20);
	if(count_tick ==0)
	{
		__HAL_TIM_SET_COUNTER(&htim2,0);
		tick_lan_truoc=0;
	}
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}
void xu_ly_tick_dht11(uint8_t* tick,uint8_t* data_dht11)
{
	HAL_TIM_Base_Stop_IT(&htim2);
	HAL_NVIC_DisableIRQ(EXTI1_IRQn);
	int count_data_dht11=0;
	for(int i =4;i<= 82;i+=2)
	{
			data_dht11[count_data_dht11/8] <<= 1;
			if( tick[i] > tick[i+1] ) {
				data_dht11[count_data_dht11/8] |= 0;
			}	else {

				data_dht11[count_data_dht11/8] |= 1;
			}
		count_data_dht11++;
	}

}
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim)
{

	if(htim->Instance == TIM2)
		{
			count++;
		}
	if(htim->Instance == TIM3)
	{
		flag_ngat_timer3=1;
		HAL_TIM_Base_Stop_IT(&htim3);
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
