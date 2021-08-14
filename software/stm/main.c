/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.cpp
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "CanFrame.h"
#include "usbd_cdc_if.h"
#include "stm32f4xx_hal_tim.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef uint8_t bool;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define true 1
#define false 0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

DAC_HandleTypeDef hdac;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

CAN_HandleTypeDef* 	canHandle;
uint32_t        	TxMailbox;
ClusterData			clusterData;
bool				needUpdate, needSend, validData, ignOverride;
uint8_t 			ignCount;

CanFrame 			rpmData;
CanFrame			tempData;
CanFrame			gearData;
CanFrame			epsData;
CanFrame			fuelCo;

//char inBuffer[16];
//uint8_t bufLoc = 0;

/*uint8_t buf1[] = {'S','e','n','d','i','n','g',' ','C','A','N',' ','m','e','s','s','a','g','e','.','.','.','\n','\r'};
uint8_t buf2[] = "Got message!\n\r";
CanFrame testData;*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM7_Init(void);
static void MX_SPI2_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

void processData(uint8_t* Buf, uint32_t *Len);
bool calcPacketHash();
void updateData();
void sendData();
void clusterInit();
void errorHandler(uint8_t err);
//void delay_us (uint16_t us);

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
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM7_Init();
  MX_SPI2_Init();
  MX_DAC_Init();
  MX_TIM8_Init();
  MX_TIM13_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  registerReceiveCallback(processData);
  clusterInit();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //uint32_t msg = 1;
  //HAL_SPI_Transmit(&hspi2, &msg, 3, 100);
  TIM8->ARR = 65535;
  //14 = 170.896Hz, 13 = 183.104Hz | 140MPH ~= 175Hz
  //2.50247959 kilohertz / ARR = Output frequency

  /*testData.TxHeader.StdId =	1284;
    testData.TxHeader.ExtId = 0x01;
    testData.TxHeader.RTR = 	CAN_RTR_REMOTE;
    testData.TxHeader.IDE = 	CAN_ID_STD;
    testData.TxHeader.DLC = 	8;
    testData.TxHeader.TransmitGlobalTime = 	DISABLE;
    testData.data.bytes[0] = 0x00;
    testData.data.bytes[1] = 0x00;
    testData.data.bytes[2] = 0x00;
    testData.data.bytes[3] = 0x00;
    testData.data.bytes[4] = 0x00;
    testData.data.bytes[5] = 0x00;
    testData.data.bytes[6] = 0x00;
    testData.data.bytes[7] = 0x00;*/

  while (1) {
	  if (needUpdate) {
		  validData = calcPacketHash();
		  if (validData) {
			  updateData();
		  }
	  }
	  if (needSend) {
		  sendData();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLQ;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /*CAN_FilterTypeDef sFilterConfig;
  //sFilterConfig.FilterNumber = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = 0;
  sFilterConfig.FilterActivation = ENABLE;
  //sFilterConfig.BankNumber = 0;
  if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
	  Error_Handler();
  }*/

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
      /* Start Error */
      Error_Handler();
    }

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  //LL_DAC_Enable(hdac.Instance, DAC_CHANNEL_1);
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  DAC1->DHR12R1 = 0;

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 30000;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1400;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  HAL_TIM_Base_Start_IT(&htim6);
  __HAL_TIM_DISABLE(&htim6);

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 8000-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1050-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  if (HAL_TIM_Base_Start_IT(&htim7) != HAL_OK) {
	  Error_Handler();
  }
  //TIM7->CR1 |= TIM_CR1_UDIS;
  //TIM7->DIER |= TIM_DIER_UIE;
  //TIM7->CR1 |= TIM_CR1_CEN;

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 1000;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 8;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /*if (__HAL_TIM_ENABLE_IT(&htim8, TIM_IT_UPDATE) != HAL_OK) {
	  Error_Handler();
  }*/

  if (HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 1;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 20;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim13, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 10;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  HAL_TIM_Base_Start(&htim13);
  HAL_TIM_OnePulse_Start(&htim13, TIM_CHANNEL_1);

  /* USER CODE END TIM13_Init 2 */
  HAL_TIM_MspPostInit(&htim13);

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

uint16_t getSpeed(float speed) {
	uint8_t section = speed / 2.5;
	switch (section) {
	case 0:
		return -21214*speed+65535;
	case 1:
		return -1240*speed+15600;
	case 2:
		return -840*speed+13600;
	case 3:
		return -470*speed+10825;
	case 4:
		return -370*speed+9825;
	case 5:
		return -256*speed+8400;
	case 6:
		return -224*speed+7920;
	case 7:
		return -152*speed+6660;
	case 8:
		return -168*speed+6980;
	case 9:
		return -108*speed+5630;
	case 10:
		return -116*speed+5830;
	case 11:
		return -76*speed+4730;
	case 12:
		return -80*speed+4850;
	case 13:
		return -64*speed+4330;
	case 14:
		return -56*speed+4050;
	case 15:
		return -44*speed+3600;
	case 16:
		return -50*speed+3840;
	case 17:
		return -34*speed+3160;
	case 18:
		return -34*speed+3160;
	case 19:
		return -30*speed+2970;
	case 20:
		return -30*speed+2970;
	case 21:
		return -22*speed+2550;
	case 22:
		return -24*speed+2660;
	case 23:
		return -20*speed+2430;
	case 24:
		return -22*speed+2550;
	case 25:
		return -16*speed+2175;
	case 26:
		return -17.20*speed+2253;
	case 27:
		return -16*speed+2172;
	case 28:
		return -14.80*speed+2088;
	case 29:
		return -14*speed+2030;
	case 30:
		return -12*speed+1880;
	case 31:
		return -11.60*speed+1849;
	case 32:
		return -11.60*speed+1849;
	case 33:
		return -10*speed+1717;
	case 34:
		return -10.80*speed+1785;
	case 35:
		return -8.80*speed+1610;
	case 36:
		return -8*speed+1538;
	case 37:
		return -8*speed+1538;
	case 38:
		return -8.80*speed+1614;
	case 39:
		return -7.60*speed+1497;
	case 40:
		return -6.80*speed+1417;
	case 41:
		return -6.40*speed+1376;
	case 42:
		return -6.40*speed+1376;
	case 43:
		return -7.20*speed+1462;
	case 44:
		return -6.40*speed+1374;
	case 45:
		return -4.80*speed+1194;
	case 46:
		return -4.80*speed+1194;
	case 47:
		return -4.80*speed+1194;
	case 48:
		return -5.20*speed+1242;
	case 49:
		return -4.80*speed+1193;
	case 50:
		return -4.40*speed+1143;
	case 51:
		return -4.40*speed+1143;
	case 52:
		return -4.40*speed+1143;
	case 53:
		return -4*speed+1090;
	case 54:
		return -4*speed+1090;
	case 55:
		return -4*speed+1090;
	default:
		return 3.79*speed;
	}
}

void processData(uint8_t* Buf, uint32_t *Len) {
	/*for (uint8_t i = 0; i < *Len; ++i) {
		if (Buf[i] != '\n' && Buf[i] != '\r') {
			inBuffer[bufLoc++] = Buf[i];
		} else {
			inBuffer[bufLoc++] = '\n';
			inBuffer[bufLoc++] = '\r';
			inBuffer[bufLoc] = 0;
			DAC1->DHR12R1 = atoi(inBuffer);
			//CDC_Transmit_FS(buf1, 24);

			if (HAL_CAN_AddTxMessage(&hcan1, &testData.TxHeader, testData.data.bytes, &TxMailbox) != HAL_OK) {
				if (!(hcan1.ErrorCode & HAL_CAN_ERROR_PARAM)) {
					errorHandler(1);
				}
			}
			CDC_Transmit_FS(inBuffer, bufLoc);
			bufLoc = 0;
		}
	}*/
	for (uint32_t i = 0; i < *Len; ++i) {
		clusterData.data.bytes[clusterData.loc++] = Buf[i];
		if (clusterData.loc > 95) {
			//calcPacketHash();
			needUpdate = true;
			clusterData.loc = 0;
		}
	}
}

bool calcPacketHash() {
	clusterData.hashCalc = XXH32(clusterData.data.bytes, 92, 0xebac6cdb); //hash is a uint32. Use a random seed (same for client)
	return clusterData.hashCalc == clusterData.data.hashRecv;
}

void updateData() {
	//Update rpm
	uint32_t rpm = clusterData.data.rpm * 4;
	rpmData.data.bytes[2] = rpm & 0xFF;
	rpmData.data.bytes[3] = (rpm & ~0xFF) >> 8;

	//Update temperature
	//tempData.data.bytes[1] = (clusterData.data.engtemp + 48) * (1 + (1/3));
	tempData.data.bytes[1] = (clusterData.data.engtemp + 48) / 0.75;

	//Update current gear
	gearData.data.bytes[1] = clusterData.data.gear;

	//Update eps data here if needed

	//DAC1->DHR12R1 = 0xFFF / 2;
	__HAL_TIM_ENABLE_IT(&htim8, TIM_IT_UPDATE);

	clusterData.lights = 0x00 |
				((clusterData.data.showlights & (1 << 15)) >> 15) |		//0 	FOG
				((clusterData.data.showlights & (1 << 6)) >> 5) |		//1		Turn Right
				((clusterData.data.showlights & (1 << 5)) >> 3) |		//2		Turn Left
				((clusterData.data.showlights & (1 << 12)) >> 8) |		//4		ILL
				((clusterData.data.showlights & (1 << 1)) << 4) |		//5		High beam
				((clusterData.data.showlights & (1 << 24)) >> 18) |		//6		Ignition

				((clusterData.data.showlights & (1 << 11)) >> 3) |		//8		ESC
				((clusterData.data.showlights & (1 << 14)) >> 5) |		//9		ESC OFF
				((clusterData.data.showlights & (1 << 20)) >> 10) |	//10	TPMS*
				((clusterData.data.showlights & (1 << 21)) >> 10) |		//11	Tread*
				((clusterData.data.showlights & (1 << 16)) >> 2) |		//14	Cruise Main
				((clusterData.data.showlights & (1 << 17)) >> 2) |		//15	Cruise Set
				((clusterData.data.showlights & (1 << 2)) << 15) |		//17	Brake*
				((clusterData.data.showlights & (1 << 9)) << 9) |		//18	Battery*
				((clusterData.data.showlights & (1 << 8)) << 11) |		//19	Oil pressure*
				((clusterData.data.showlights & (1 << 19)) << 1) |		//20	Trunk
				((clusterData.data.showlights & (1 << 18)) << 3) |		//21	Door
				((clusterData.data.showlights & (1 << 10)) << 12) |	//22	ABS*
				((clusterData.data.showlights & (1 << 13)) << 10);		//23	Check Engine*

	if (!ignOverride) {
		if (clusterData.data.showlights & (1 << 23)) {
			ignOverride = true;
			//HAL_TIM_Base_Start_IT(&htim6);
			__HAL_TIM_ENABLE(&htim6);
		} else {
			/*clusterData.lights |=
					((~clusterData.data.showlights & (1 << 20)) >> 10) |	//10	TPMS
					((clusterData.data.showlights & (1 << 21)) >> 10) |		//11	Tread
					((clusterData.data.showlights & (1 << 2)) << 15) |		//17	Brake
					((clusterData.data.showlights & (1 << 9)) << 9) |		//18	Battery
					((clusterData.data.showlights & (1 << 8)) << 11) |		//19	Oil pressure
					((~clusterData.data.showlights & (1 << 10)) << 12) |	//22	ABS
					((clusterData.data.showlights & (1 << 13)) << 10);		//23	Check Engine

			epsData.data.bytes[0] = 0x00;*/
		}
	} else {
		if (clusterData.data.showlights & (1 << 22)) {
			clusterData.lightsOverride &= ~(1 << 18);
		}
		clusterData.lights |= clusterData.lightsOverride;
	}

	clusterData.lights ^= (1 << 10) | (1 << 22);

	/*if (!(clusterData.data.showlights & (1 << 22))) {
		//Engine is not on
		if (!(clusterData.data.showlights & (1 << 24))) {
			//Engine off, ignition is not on
			//TODO: Turn off cluster
		} else {
			//Engine off, ignition on, starter off
			//Currently in BeamNG, this must mean the engine stalled. Do nothing
		}
	}*/

	/*if (!ignOverride && (clusterData.data.showlights & (1 << 23))) {
		//Engine off, ignition off, starter is on

		//	Charge -	On until engine started
		//	Brake -		3 seconds solid
		//	ABS -		3 seconds solid
		//	Seat belt - 6 blinks (0.5 on then 0.5 off) (Then, another 6 after start)
		//	Airbag -	6 seconds solid
		//	Oil -		On until starter begins
		//	Check ENG - On until starter begins
		//	TPMS -		3 seconds solid
		//	Tread -		3 seconds solid
		//	EPS -		On until starter begins

		clusterData.lights |= (1 << 18) |
								(1 << 17) |
								(1 << 16) |
								(1 << 3)  |
								(1 << 19) |
								(1 << 23) |
								(1 << 10) |
								(1 << 11);
		clusterData.lights &= ~(1 << 22);
		epsData.data.bytes[0] = 0b100;
	}*/
}

void sendData() {
	//Send RPM message
	if (HAL_CAN_AddTxMessage(&hcan1, &rpmData.TxHeader, rpmData.data.bytes, &TxMailbox) != HAL_OK) {
		if (!(hcan1.ErrorCode & HAL_CAN_ERROR_PARAM)) {
			errorHandler(1);
		}
	}
	//Send temperature message
	if (HAL_CAN_AddTxMessage(&hcan1, &tempData.TxHeader, tempData.data.bytes, &TxMailbox) != HAL_OK) {
		if (!(hcan1.ErrorCode & HAL_CAN_ERROR_PARAM)) {
			errorHandler(1);
		}
	}

	//Update fuel consumption
	//60L (60000000uL) for testing on Pessima 2.7 LX V6 Sport
	/*float dFuel = clusterData.lastFuel - clusterData.data.fuel;
	clusterData.lastFuel = clusterData.data.fuel;
	if (dFuel < 0) {
		dFuel = 0;
	}
	uint16_t fuelSend = (dFuel * 60000000) / 0.128;
	fuelCo.data.bytes[1] = fuelSend & 0xFF;
	fuelCo.data.bytes[2] = (fuelSend >> 8) & 0xFF;*/

	//Send fuel message
	/*if (HAL_CAN_AddTxMessage(&hcan1, &fuelCo.TxHeader, fuelCo.data.bytes, &TxMailbox) != HAL_OK) {
		if (!(hcan1.ErrorCode & HAL_CAN_ERROR_PARAM)) {
			errorHandler(1);
		}
	}*/
	//Send gear message
	if (HAL_CAN_AddTxMessage(&hcan1, &gearData.TxHeader, gearData.data.bytes, &TxMailbox) != HAL_OK) {
		if (!(hcan1.ErrorCode & HAL_CAN_ERROR_PARAM)) {
			errorHandler(1);
		}
	}
	//Send EPS message
	if (HAL_CAN_AddTxMessage(&hcan1, &epsData.TxHeader, epsData.data.bytes, &TxMailbox) != HAL_OK) {
		if (!(hcan1.ErrorCode & HAL_CAN_ERROR_PARAM)) {
			errorHandler(1);
		}
	}

	HAL_SPI_Transmit(&hspi2, (uint8_t*)&clusterData.lights, 3, 100);
	__HAL_TIM_ENABLE(&htim13);
}

void clusterInit() {
	rpmData.TxHeader.StdId = 				0x316;
	rpmData.TxHeader.ExtId = 				0x01;
	rpmData.TxHeader.RTR = 					CAN_RTR_DATA;
	rpmData.TxHeader.IDE = 					CAN_ID_STD;
	rpmData.TxHeader.DLC = 					8;
	rpmData.TxHeader.TransmitGlobalTime = 	DISABLE;
	rpmData.data.value = 					0;

	tempData.TxHeader.StdId = 				0x329;
	tempData.TxHeader.ExtId = 				0x01;
	tempData.TxHeader.RTR = 				CAN_RTR_DATA;
	tempData.TxHeader.IDE = 				CAN_ID_STD;
	tempData.TxHeader.DLC = 				8;
	tempData.TxHeader.TransmitGlobalTime = 	DISABLE;
	tempData.data.value = 					0;

	gearData.TxHeader.StdId = 				0x43F;
	gearData.TxHeader.ExtId = 				0x01;
	gearData.TxHeader.RTR = 				CAN_RTR_DATA;
	gearData.TxHeader.IDE = 				CAN_ID_STD;
	gearData.TxHeader.DLC = 				8;
	gearData.TxHeader.TransmitGlobalTime = 	DISABLE;
	gearData.data.value = 					0;

	epsData.TxHeader.StdId = 				0x5E4;
	epsData.TxHeader.ExtId = 				0x01;
	epsData.TxHeader.RTR = 					CAN_RTR_DATA;
	epsData.TxHeader.IDE = 					CAN_ID_STD;
	epsData.TxHeader.DLC = 					3;
	epsData.TxHeader.TransmitGlobalTime = 	DISABLE;
	epsData.data.value = 					0;

	//fuelCo.TxHeader.StdId = 				0x545;
	fuelCo.TxHeader.StdId = 				880;
	fuelCo.TxHeader.ExtId = 				0x01;
	fuelCo.TxHeader.RTR = 					CAN_RTR_DATA;
	fuelCo.TxHeader.IDE = 					CAN_ID_STD;
	fuelCo.TxHeader.DLC = 					8;
	fuelCo.TxHeader.TransmitGlobalTime = 	DISABLE;
	fuelCo.data.value = 					0;
	//fuelCo.data.bytes[0] =					0xE2;
	//fuelCo.data.bytes[3] =					0x7A;
	//fuelCo.data.value = 0x000000008C004EE0;
	fuelCo.data.value = 0x80000000;

	clusterData.loc = 0;
	clusterData.lastFuel = 0;
	for (uint8_t i = 0; i < 96; ++i) {
		clusterData.data.bytes[i] = 0;
	}

	needUpdate = false;
	needSend = false;
	validData = false;
	ignOverride = false;

	ignCount = 0;

	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
}

void errorHandler(uint8_t err) {
	__disable_irq();
	switch (err) {
	case 1:
		while (1) {
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			HAL_Delay(100);
		}
	default:
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		while (1) {
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			HAL_Delay(500);
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	//Prescaler * Overflow = Period-in-Cycles = Clock-cycles-per-Second / Frequency
	if (htim == &htim7) {
		needSend = true;
	} else if (htim == &htim8) {
		TIM8->ARR = getSpeed(clusterData.data.speedMS * 2.236936);
		//TIM8->EGR |= TIM_EGR_UG;
		__HAL_TIM_DISABLE_IT(&htim8, TIM_IT_UPDATE);
	} else if (htim == &htim6) {
		//	Charge -	On until engine started
		//	Brake -		3 seconds solid
		//	ABS -		3 seconds solid
		//	Seat belt - 6 blinks (0.5 on then 0.5 off) (Then, another 6 after start)
		//	Airbag -	6 seconds solid
		//	Oil -		On until starter begins
		//	Check ENG - On until starter begins
		//	TPMS -		3 seconds solid
		//	Tread -		3 seconds solid
		//	EPS -		On until starter begins
		if (ignCount == 0) {
			clusterData.lightsOverride |=
							(1 << 17) |
							(1 << 11) |
							(1 << 22) |
							(1 << 10) |

							(1 << 3) |

							(1 << 18);
			//epsData.data.bytes[0] = 0b100;
		} else if (ignCount == 5) {
			clusterData.lightsOverride &=
							~((1 << 17) |
							(1 << 11) |
							(1 << 22) |
							(1 << 10));
		}
		clusterData.lightsOverride ^= (1 << 16);

		++ignCount;
		if (ignCount >= 11) {
			clusterData.lightsOverride &= ~(1 << 3);

			__HAL_TIM_DISABLE(&htim6);
			ignOverride = false;
			ignCount = 0;
		}
	}
}

/*void delay_us (uint16_t us) {
	__HAL_TIM_SET_COUNTER(&htim1,0);  						// set the counter value a 0
	while ((uint16_t)__HAL_TIM_GET_COUNTER(&htim1) < us);	// wait for the counter to reach the us input in the parameter
}*/

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
