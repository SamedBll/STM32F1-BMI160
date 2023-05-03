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
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t vol_data0[7][2] = {{0},{0}};
int16_t data0[7] = {0};
char RxData[1];
double gyr_x = .0;
double gyr_y = .0;
double gyr_z = .0;
double acc_x = .0;
double acc_y = .0;
double acc_z = .0;
uint16_t timestamp = 0;
uint16_t old_timestamp = 0;
int delta_time = 0;
double imu_data[7] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void idleReadSPI(void);
int8_t chipID(void);
int8_t BMI160Read_Return(uint8_t address, size_t size, int timeout);
void BMI160Read(uint8_t *address, uint8_t *reply, size_t size, int timeout);
void BMI160Write(uint8_t address, uint8_t pValue,  int timeout);
void BMI160Init(void);
void foc(void);
int8_t offset_samples(int8_t *off2_vals);
int8_t BMI160R_Rtr_WDI(uint8_t address, size_t size, int timeout);
void delay_us(uint16_t us);
void BMI160_GetData(uint8_t address, uint8_t *vol_data[][2], size_t size, int timeout);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  HAL_UART_Receive_IT(&huart1, RxData, 1);
  if(RxData[0] == '1'){
	  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,75); //Second motor 75% voltage
	  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,75);
  }
  else if(RxData[0] == '0'){
	  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0); //Second motor 75% voltage
	  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
   HAL_UART_Transmit_IT(&huart1, imu_data, sizeof (imu_data));
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
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start(&htim4);               //Initialize stm32 timer 3
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);  //PB0 Start pwm second motor 100% duty cycle
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);

  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);   // Start first motor clock wise rotation
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);   //Start second motor clock wise rotation
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);


  HAL_UART_Receive_IT (&huart1, RxData, 1);
  HAL_UART_Transmit_IT(&huart1, imu_data, sizeof (imu_data));

  while(chipID()!=1){}
  HAL_Delay(100);
  BMI160Init();
  HAL_Delay(100);
  //-----"By burst reading FIFO data Register 0x24, data is read from FIFO every 50ms."-----//
  //----- Bu nedenle FIFO kullanılmadı zaten sensör kısıtlaması normal mod için min 2us.---//

//  uint8_t cmd_w = 0x7E;
//  uint8_t softreset = 0xB6;
//  uint8_t magif1 = BMI160Read_Return(0xCC,1,100);
//  HAL_Delay(100);
//
//  int8_t pmu_status = BMI160Read_Return(0x83,1,100);
//  HAL_Delay(100);

  //------------------Error Check------------------//
  uint8_t err = (uint8_t)BMI160Read_Return(0x82,1,100);
  HAL_Delay(100);

  foc();

/*  uint8_t vol_data1[7][2] = {{0},{0}};
  int16_t data1[7] = {0};

  BMI160_GetData(0x8C, &vol_data0,7,10);
  BMI160_GetData(0x8C, &vol_data1,7,10);

  for(int i = 0; i<7; ++i){
  		data0[i] = (vol_data0[i][1]<<8) | (vol_data0[i][0]);
  	}

  for(int i = 0; i<7; ++i){
    		data1[i] = (vol_data1[i][1]<<8) | (vol_data1[i][0]);
    }*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //data0[0]= gyr_x
	  //data0[1]= gyr_y
	  //data0[2]= gyr_z
	  //data0[3]= acc_x
	  //data0[4]= acc_y
	  //data0[5]= acc_z
	  //data0[6]= timestamp
	  BMI160_GetData(0x8C, &vol_data0,7,10);
	  for(size_t i = 0; i<7; ++i){
			data0[i] = (vol_data0[i][1]<<8) | (vol_data0[i][0]);
	  }

	  for(size_t i = 0; i<3; ++i){
	  		imu_data[i] = (250.0/65536.0)*data0[i];
	  }
	  for(size_t i = 3; i<6; ++i){
	  	  	imu_data[i] = (32.0/65536.0)*data0[i];
	  }
	  gyr_x = (250.0/65536.0)*data0[0];
	  gyr_y = (250.0/65536.0)*data0[1];
	  gyr_z = (250.0/65536.0)*data0[2];
	  acc_x = (32.0/65536.0)*data0[3];
	  acc_y = (32.0/65536.0)*data0[4];
	  acc_z = (32.0/65536.0)*data0[5];
	  timestamp = data0[6];
	  if(timestamp - old_timestamp >0){
		  //delta_time = timestamp-old_timestamp;
		  imu_data[6] = (double)timestamp-old_timestamp;
	  }else{
		  //delta_time = (65535-old_timestamp)+timestamp;
		  imu_data[6] =(double)(65535.0-old_timestamp)+timestamp;
	  }
	  old_timestamp = timestamp;

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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL5;
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
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 40-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 40-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 230400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB4 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void idleReadSPI(void){
	//BMI160'a SPI başlatıldığını haber vermek için, NSS pinini low-high toggle yapılmalı
	uint8_t idle_address[1] = {0xFF};
	uint8_t idle_reply[1] = {0};
	HAL_SPI_Init(&hspi1);
	HAL_SPI_Transmit(&hspi1, idle_address, 1, 100);
	HAL_SPI_Receive(&hspi1, idle_reply,1,100);
	HAL_SPI_DeInit(&hspi1);
}

int8_t chipID(void){
	//BMI160 Basic Kontrol
	idleReadSPI();
	uint8_t address = 0x80;
	uint8_t reply = 0;
	HAL_SPI_Init(&hspi1);
	HAL_SPI_Transmit(&hspi1, &address, 1, 100);
	HAL_SPI_Receive(&hspi1, &reply,1,100);
	HAL_SPI_DeInit(&hspi1);
	return ((uint8_t)reply==0xD1) ? 1 : 0;
}

int8_t BMI160Read_Return(uint8_t address, size_t size, int timeout){
	uint8_t reply = 0;
	HAL_SPI_Init(&hspi1);
	HAL_SPI_Transmit(&hspi1, &address, size, timeout);
	HAL_SPI_Receive(&hspi1, &reply, size, timeout);
	HAL_SPI_DeInit(&hspi1);
	return reply;
}


void BMI160_GetData(uint8_t address, uint8_t *vol_data[][2], size_t size, int timeout){
	HAL_SPI_Init(&hspi1);
	HAL_SPI_Transmit(&hspi1, &address, 1, timeout);
	HAL_SPI_Receive(&hspi1, vol_data, size*2, timeout);
	HAL_SPI_DeInit(&hspi1);
}


void BMI160Write(uint8_t address, uint8_t pValue,  int timeout){
	HAL_SPI_Init(&hspi1);
	HAL_SPI_Transmit(&hspi1, &address, 1, timeout);
	HAL_SPI_Transmit(&hspi1, &pValue, 1, timeout);
	HAL_SPI_DeInit(&hspi1);
}

void BMI160Init(void){
	BMI160Write(0X4C,0x00,100); //mag_manuel_en = 0
	HAL_Delay(100);
	BMI160Write(0x7E,0b00010001,100); //acc_pmu_status = normal
	HAL_Delay(100);
	BMI160Write(0x7E,0b00010101,100); //gyr_pmu_status = normal
	HAL_Delay(100);
	BMI160Write(0x7E,0b00011000,100); //mag_pmu_status = suspend
	HAL_Delay(100);
	BMI160Write(0x40,0b00001100,100); //acc_conf = OSR4, ODR 1600HZ -> Bu kısım değiştirilip gözlem yapılabilir
	HAL_Delay(100);
	BMI160Write(0x41,0b00001100,100); //acc_range = ±16g range
	HAL_Delay(100);
	BMI160Write(0x42,0b00001100,100); //gyr_conf = OSR4, ODR 1600HZ -> Bu kısım değiştirilip gözlem yapılabilir
	HAL_Delay(100);
	BMI160Write(0x43,0b00000100,100); //gyr_range = ±125°/s
	HAL_Delay(100);
}

void foc(void){
	HAL_Delay(100);
	BMI160Write(0x77,0b11000000,100); //gyr_off_en = 1, acc_off_en = 1;
	HAL_Delay(100);
	BMI160Write(0x69,0b01111101,100); //foc_gyr_en = 1, foc_acc_x = 11, foc_acc_y = 11, foc_acc_z = 01
	HAL_Delay(100);
	BMI160Write(0x7E,0b00000011,100); //CMD->start_foc
	HAL_Delay(1000);
	uint8_t status= (uint8_t)BMI160Read_Return(0x9B,1,100);
	HAL_Delay(1);
	uint8_t foc_rdy = (1<<3);
	while((status&foc_rdy)!=0x08){
		status= (uint8_t)BMI160Read_Return(0x9B,1,100);
		HAL_Delay(1);
	}
}


int8_t offset_samples(int8_t *off2_vals){
	//Gürültülü ortamda offset değerini en iyi belirleyebilmek için belirli aralıklarla sample
	//alınması için yazıldı. Alınan sample'lar median filtresinden geçirilip sonra elde edilen
	//verilerin ortalaması, medyanı veya modu hesaplanıp offset değeri olarak kullanılabilir.
	//Ya da kullanıcı offset değerini ayarlayabilir; z ekseni için; z ekseninde alınan raw verileri
	//işleme sokarak ona uygun bir offset değeri bulunabilir. raw verinin modundan 1G'ye eşitlenmesi
	//için çıkarılması gereken değer offset verisi olabilir.
	HAL_Delay(10);
	BMI160Write(0x69,0b01111101,100); //foc_gyr_en = 1, foc_acc_x = 11, foc_acc_y = 11, foc_acc_z = 01
	HAL_Delay(10);
	for(int i = 0; i<10; ++i){ //10 sample
		BMI160Write(0x7E,0b00000011,100); //CMD->start_foc
		HAL_Delay(1000);
		uint8_t status= (uint8_t)BMI160Read_Return(0x9B,1,100);
		HAL_Delay(1);
		uint8_t foc_rdy = (1<<3);
		while((status&foc_rdy)!=0x08){
			status= (uint8_t)BMI160Read_Return(0x9B,1,100);
		}
		int8_t offset2 = BMI160Read_Return(0xF3,1,100);
		*off2_vals = offset2;
		++off2_vals;
	}
	return 1;
}

void delay_us(uint16_t us){
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
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
