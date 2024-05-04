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
#include "LCD.h"
#include <string.h>
#include <stdio.h>
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
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
int state_lcd =0;
char mat_khau_dung[6] = {1,2,3,4,5,6};
char mat_khau_user[6] = {0,0,0,0,0,0};
int check_pass_LCD =0;
int count_mat_khau=0;
int count_may_chu_nhap_vao=0;
int flag_di_qua_nhap_mat_khau =0;
int flag_chuyen_lcd=0;
int flag_lan_dau_nhap_mat_khau =0;


enum week{Mo , Tu, We, Th, Fr , Sa, Su};


int ngay=0;
int ngay_bao_thuc =0;
int thang=0;
int thang_bao_thuc=0;
int nam=0;
int nam_bao_thuc=0;
int gio=0;
int gio_bao_thuc=0;
int phut=0;
int phut_bao_thuc=0;
int giay=0;
int giay_bao_thuc=0;
int bao_nhiet_do =0;
char ngay_string[20];
char gio_string[20];




#define DS3231_ADDRESS 0xD0

// Convert normal decimal numbers to binary coded decimal
uint8_t decToBcd(int val)
{
  return (uint8_t)( (val/10*16) + (val%10) );
}
// Convert binary coded decimal to normal decimal numbers
int bcdToDec(uint8_t val)
{
  return (int)( (val/16*10) + (val%16) );
}

typedef struct {
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hour;
	uint8_t dayofweek;
	uint8_t dayofmonth;
	uint8_t month;
	uint8_t year;
} TIME;

TIME time;

// function to set time

void Set_Time (uint8_t sec, uint8_t min, uint8_t hour, uint8_t dow, uint8_t dom, uint8_t month, uint8_t year)
{
	uint8_t set_time[7];
	set_time[0] = decToBcd(sec);
	set_time[1] = decToBcd(min);
	set_time[2] = decToBcd(hour);
	set_time[3] = decToBcd(dow);
	set_time[4] = decToBcd(dom);
	set_time[5] = decToBcd(month);
	set_time[6] = decToBcd(year);

	HAL_I2C_Mem_Write(&hi2c1, DS3231_ADDRESS, 0x00, 1, set_time, 7, 1000);
}

void Get_Time (void)
{
	uint8_t get_time[7];
	HAL_I2C_Mem_Read(&hi2c1, DS3231_ADDRESS, 0x00, 1, get_time, 7, 1000);
	time.seconds = bcdToDec(get_time[0]);
	time.minutes = bcdToDec(get_time[1]);
	time.hour = bcdToDec(get_time[2]);
	time.dayofweek = bcdToDec(get_time[3]);
	time.dayofmonth = bcdToDec(get_time[4]);
	time.month = bcdToDec(get_time[5]);
	time.year = bcdToDec(get_time[6]);
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  lcd_init();
  lcd_clear();
  HAL_TIM_Base_Stop_IT(&htim2);
  HAL_TIM_Base_Stop_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);
  Set_Time(20, 30, 20, 5, 26, 4, 24);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // lưu ý khi cách đ�?c dht11 là dùng ngắt ngoài nên sử dụng ngắt ưu tiên cao nhất
	  // trước khi đ�?c dht11 thì g�?i hàm doc_dht11() sau đó delay 10ms và g�?i hàm xy_ly_tick_dht11
		doc_dht11();
		HAL_Delay(10);
		xu_ly_tick_dht11(tick,data_dht11); // hàm này lấy pointer cho ra data_dht11 là 1 mảng với
		/* tương ứng 	data_dht11[0] là phần nguyên độ ẩm
						data_dht11[0] là phần thập phân độ ẩm
						data_dht11[0] là phần nguyên nhiệt độ
						data_dht11[0] là phần thập phân nhiệt độ
						*/
		Get_Time();
		sprintf(nhiet_do,"Nhiet do: %d.%doC", data_dht11[2], data_dht11[3]); // hàm này ghép chuỗi thành chuỗi nhiet_do để tiện hiển thị LCD
		sprintf(do_am, "Do am:  %d.%d ", data_dht11[0],data_dht11[1]);		// hàm ghép chuỗi độ ẩm để hiển thị LCD

		bool a = (ngay == ngay_bao_thuc)? 1:0 ;
		bool b = (thang == thang_bao_thuc)? 1:0 ;
		bool c = (nam == nam_bao_thuc)? 1:0 ;
		bool d = (gio == gio_bao_thuc)? 1:0 ;
		bool e = (phut == phut_bao_thuc)? 1:0 ;
		bool flag_bat_bao_thuc = a&b&c&d&e;
		if(flag_bat_bao_thuc)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 1);
		} else {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 0);
		}
		bool flag_bat_nhiet_do = (data_dht11 >= bao_nhiet_do)? 1:0;
		if(flag_bat_nhiet_do)
		{
			// bật báo nhiệt độ
		}
//		if( ngay == ngay_bao_thuc) {
//
//		}
		if(flag_ngat_timer3)
		{
			flag_ngat_timer3=0;
			switch(state_lcd%7) {
				case 0:
				{
					  lcd_put_cur(0,1);
					  lcd_send_string(nhiet_do);
					  lcd_put_cur(1,1);
					  lcd_send_string(do_am);
					  break;
				}
				case 1:
				{

					if(flag_chuyen_lcd)
					{
						count_mat_khau=-2;
						count_may_chu_nhap_vao=0;
						flag_lan_dau_nhap_mat_khau =1;
						flag_chuyen_lcd=0;
						lcd_clear();
						lcd_put_cur(0,1);
						lcd_send_string("Nhap mat khau:");
						lcd_put_cur(1,0);
					}
						count_mat_khau++;
						count_may_chu_nhap_vao++;
						if((flag_number <= 9) && (flag_number >=0))
						{
							mat_khau_user[count_mat_khau] = flag_number;
						}
						  switch(flag_number) {
							  case 11:
							  {
								  for(int i=0 ;i < 6 ; i++)
									{
										if(mat_khau_user[i] != mat_khau_dung[i])
										{
											check_pass_LCD=0;
											break;
										}
										check_pass_LCD=1;
									}
									for(int i=0;i<6;i++)
									{
										mat_khau_user[i] =0;
									}
									if(check_pass_LCD) {
										state_lcd =2;
										flag_ngat_timer3=1;
										lcd_clear();
										lcd_put_cur(0,1);
										lcd_send_string("Mat khau dung ->>");
										flag_di_qua_nhap_mat_khau=1;
										HAL_Delay(2000);
										flag_number=0;
									} else {
										state_lcd =1;
										flag_chuyen_lcd=1;
										flag_ngat_timer3=1;
										lcd_clear();
										lcd_put_cur(0,1);
										lcd_send_string("Mat khau sai ");
										lcd_put_cur(1,1);
										lcd_send_string("Moi nhap lai  ");
										HAL_Delay(2000);
										flag_number=0;
									}
								  break;
							  }
							  default :
							  {
								  if(count_mat_khau <0)
								  {

								  } else {

								  lcd_send_string("*");
								  }
								  break;
							  }
						  }
					  break;
				}

				case 3:
				{
					  lcd_clear();
					  lcd_put_cur(0,1);
					  lcd_send_string("set ngay");
					  break;
				}
				case 4:
				{
					lcd_clear();
					  lcd_put_cur(0,1);
					  lcd_send_string("set gio");
					  break;
				}
				case 5:
				{
					lcd_clear();
					  lcd_put_cur(0,1);
					  lcd_send_string("cai bao thuc");
					  break;
				}
				case 6:
				{
					lcd_clear();
					  lcd_put_cur(0,1);
					  lcd_send_string("cai bao nhiet do");
					  break;
				}

			}
		}

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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 63000;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
							if(state_lcd ==1)
							{
								if(flag_di_qua_nhap_mat_khau==1)
								{
									state_lcd ++;
								} else {

								}
							} else {
								state_lcd ++;
							}
							flag_chuyen_lcd=1;
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
	if(htim->Instance == TIM4)
	{
		if(state_lcd%7 == 2){
			  sprintf(gio_string,"%d : %d : %d ",time.hour,time.minutes,time.seconds);
			  sprintf(ngay_string,"%d : %d : 20%d",time.dayofmonth,time.month,time.year);
			  lcd_clear();
			  lcd_put_cur(0,1);
			  lcd_send_string(ngay_string);
			  lcd_put_cur(1,1);
			  lcd_send_string(gio_string);
		}
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
