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
#include "VL53L1X_api.h"
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
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t dev = 0x52;

uint8_t nof = 6;
uint16_t* dis = NULL;

uint8_t devs[9] = {
		0x54,
		0x56,
		0x58,
		0x5A,
		0x5C,
		0x5E,
		0x60,
		0x62,
		0x64
};

CAN_FilterTypeDef canFilter1;
CAN_RxHeaderTypeDef can1RxHeader;
CAN_TxHeaderTypeDef can1TxHeader;

uint8_t can1Rx0Data[8];
uint32_t TxMailBox1;
uint8_t can1Tx0Data[8];
volatile uint8_t can1_rx0_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
void TurnOffAll(void);
uint8_t TurnOnAt(uint8_t i);
uint16_t ChangeAddresses(uint8_t num_of_tof);
uint8_t GetAllData(uint8_t num_of_tof, uint16_t dis[num_of_tof]);
void PrintAllData(uint8_t num_of_tof, uint16_t dis[num_of_tof]);

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void CAN_Filter_Init(void);
void CAN_Send(uint8_t ID, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char* p, int len) {
	HAL_UART_Transmit(&huart2, p, len, 16);
	return len;
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
  dis = calloc(nof, 16 * nof);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  HAL_NVIC_DisableIRQ(EXTI1_IRQn);

  CAN_Filter_Init();

//	uint8_t bootState = 0;
//	uint8_t boot[8] = {66, 79, 79, 84, 48, 13, 10, 0};
//	while(bootState==0){
//		VL53L1X_BootState(dev, &bootState);
//		HAL_UART_Transmit(&huart2, boot, 8, 10);
//		HAL_Delay(2);
//	}
//	HAL_Delay(5);
//	VL53L1X_SensorInit(dev);
//	VL53L1X_SetInterruptPolarity(dev, 0);
//	VL53L1X_StartRanging(dev);

  if (!(ChangeAddresses(nof))) {
	  //HAL_UART_Transmit(&huart2, fail, 6, 10);
  }
  else {
	  //HAL_UART_Transmit(&huart2, succ, 6, 10);
  }
  VL53L1X_SetInterruptPolarity(devs[0], 0);
  VL53L1X_ClearInterrupt(devs[0]);
  HAL_Delay(100);
  	  GetAllData(nof, dis);
  	  PrintAllData(nof, dis);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //HAL_TIM_Base_Start_IT(&htim7);
  while (1)
  {
//	  while (dataReady == 0){
//		  VL53L1X_CheckForDataReady(dev, &dataReady);
//		  //HAL_UART_Transmit(&huart2, getting, 6, 10);
//		  HAL_Delay(2);
//	  }
//	  VL53L1X_GetDistance(dev, &dis[0]);
//	  	  VL53L1X_ClearInterrupt(dev);
//	  	printf("%d\r\n", dis[0]);
//	  		  dataReady = 0;
//	  		  HAL_Delay(50);

//	  CAN_Send(0x10, (dis[0] & 0xFF00) >> 8, (dis[0] & 0xFF), (dis[1] & 0xFF00) >> 8, (dis[1] & 0xFF), (dis[2] & 0xFF00) >> 8, (dis[2] & 0xFF), 7, 0);
//
//	  if (can1_rx0_flag == 1) {
//		  can1_rx0_flag = 0;
//			printf("%d%d\r\n", 0, 1);
//		  //HAL_UART_Transmit(&huart2, outer, 6, 10);
//	  }
//	  GetAllData(nof, dis);
//	  PrintAllData(nof, dis);
//	  HAL_Delay(400);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
  hcan1.Init.Prescaler = 50;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 400000;
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
  htim7.Init.Prescaler = 720-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1000-1;
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

  /* USER CODE END TIM7_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC0 PC2 PC3
                           PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */
void TurnOffAll(void) {
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
}

uint8_t TurnOnAt(uint8_t i) {
	uint8_t bootState = 0;
	uint8_t boot[8] = {66, 79, 79, 84, 48, 13, 10, 0};
	switch (i) {
	case 0:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
		HAL_Delay(10);
		while(bootState==0){
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
			HAL_Delay(10);
			VL53L1X_BootState(dev, &bootState);
			HAL_UART_Transmit(&huart2, boot, 8, 10);
			HAL_Delay(2);
		}
		break;
	case 1:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
		HAL_Delay(10);
		boot[4] = 49;
		while(bootState==0){
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
			HAL_Delay(10);
			VL53L1X_BootState(dev, &bootState);
			HAL_UART_Transmit(&huart2, boot, 8, 10);
			HAL_Delay(2);
		}
		break;
	case 2:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
		HAL_Delay(10);
		boot[4] = 50;
		while(bootState==0){
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
			HAL_Delay(10);
			VL53L1X_BootState(dev, &bootState);
			HAL_UART_Transmit(&huart2, boot, 8, 10);
			HAL_Delay(2);
		}
		break;
	case 3:
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
		HAL_Delay(10);
		boot[4] = 51;
		while(bootState==0){
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
			HAL_Delay(10);
			VL53L1X_BootState(dev, &bootState);
			HAL_UART_Transmit(&huart2, boot, 8, 10);
			HAL_Delay(2);
		}
		break;
	case 4:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
		HAL_Delay(10);
		boot[4] = 52;
		while(bootState==0){
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			HAL_Delay(10);
			VL53L1X_BootState(dev, &bootState);
			HAL_UART_Transmit(&huart2, boot, 8, 10);
			HAL_Delay(2);
		}
		break;
	case 5:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
		HAL_Delay(10);
		boot[4] = 53;
		while(bootState==0){
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
			HAL_Delay(10);
			VL53L1X_BootState(dev, &bootState);
			HAL_UART_Transmit(&huart2, boot, 8, 10);
			HAL_Delay(2);
		}
		break;
	case 6:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
		HAL_Delay(10);
		boot[4] = 54;
		while(bootState==0){
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
			HAL_Delay(10);
			VL53L1X_BootState(dev, &bootState);
			HAL_UART_Transmit(&huart2, boot, 8, 10);
			HAL_Delay(2);
		}
		break;
	case 7:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
		HAL_Delay(10);
		boot[4] = 55;
		while(bootState==0){
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
			HAL_Delay(10);
			VL53L1X_BootState(dev, &bootState);
			HAL_UART_Transmit(&huart2, boot, 8, 10);
			HAL_Delay(2);
		}
		break;
	case 8:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
		HAL_Delay(10);
		boot[4] = 56;
		while(bootState==0){
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
			HAL_Delay(10);
			VL53L1X_BootState(dev, &bootState);
			HAL_UART_Transmit(&huart2, boot, 8, 10);
			HAL_Delay(2);
		}
		break;
	default:
		;
		// shit
	}
	return bootState;
}

uint16_t ChangeAddresses(uint8_t num_of_tof) {
	uint8_t success = 1;
	TurnOffAll();
	HAL_Delay(10);
	for (uint8_t i = 0; i != num_of_tof; ++i) {
		success *= TurnOnAt(i);
		HAL_Delay(5);
		printf("try init: %d\r\n", i);
		VL53L1X_SensorInit(dev);
		printf("try addr change: %d\r\n", i);
		VL53L1X_SetI2CAddress(dev, devs[i]);
		VL53L1X_StartRanging(devs[i]);
		printf("done: %d\r\n", i);
	}
	return success;
}

uint8_t GetAllData(uint8_t num_of_tof, uint16_t dis[num_of_tof]) {
	//printf("getting %d\r\n", 0);
	//HAL_Delay(2);
	//printf("WTF1 %d\r\n", 0);
	VL53L1X_GetDistance(devs[0], &dis[0]);
	//printf("WTF2 %d\r\n", 0);
	VL53L1X_ClearInterrupt(devs[0]);
	//printf("WTF3 %d\r\n", 0);

	uint8_t dataReady = 0;
	for (uint8_t i = 1; i != num_of_tof; ++i) {
		dataReady = 0;
		while (dataReady == 0){
			VL53L1X_CheckForDataReady(devs[i], &dataReady);
			//printf("getting %d\r\n", i);
			//HAL_Delay(2);
		}
		VL53L1X_GetDistance(devs[i], &dis[i]);
		VL53L1X_ClearInterrupt(devs[i]);
	}
	//HAL_Delay(50);
}

void PrintAllData(uint8_t num_of_tof, uint16_t dis[num_of_tof]) {
	for (int i = 0; i != num_of_tof; ++i) {
		printf("%d:%d\r\n", i, dis[i]);
	}
	//HAL_Delay(500);
}

void CAN_Filter_Init(void)
{
	//Set the CAN Filter which has Mask Ids
	//CAN Filter1
	//Receiving CAN data via 0x102~0x10E
//	canFilter1.FilterMaskIdHigh = 0x7F3 << 5; // Shift 5 bit
//	canFilter1.FilterIdHigh = 0x106 << 5;
//	canFilter1.FilterMaskIdLow = 0x7F3 << 5; // Shift 5 bit
//	canFilter1.FilterIdLow = 0x106 << 5;
	canFilter1.FilterMaskIdHigh = 0x000 << 5; // Shift 5 bit
	canFilter1.FilterIdHigh = 0x000 << 5;
	canFilter1.FilterMaskIdLow = 0x000 << 5; // Shift 5 bit
	canFilter1.FilterIdLow = 0x000 << 5;
	canFilter1.FilterMode = CAN_FILTERMODE_IDMASK;
	canFilter1.FilterScale = CAN_FILTERSCALE_16BIT;
	canFilter1.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	canFilter1.FilterBank = 0;
	canFilter1.FilterActivation = ENABLE;

	/* Set options for messages; ID type(standard), Length(8 byte) */
	can1TxHeader.RTR = CAN_RTR_DATA;
	can1TxHeader.IDE = CAN_ID_STD;
	can1TxHeader.DLC = 8;

	HAL_CAN_ConfigFilter(&hcan1, &canFilter1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_Start(&hcan1);
}

void CAN_Send(uint8_t ID, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7)
{
	  /* Set options for messages; Address */
	  can1TxHeader.StdId = ID;

	  can1Tx0Data[7] = data7;
	  can1Tx0Data[6] = data6;
	  can1Tx0Data[5] = data5;
	  can1Tx0Data[4] = data4;
	  can1Tx0Data[3] = data3;
	  can1Tx0Data[2] = data2;
	  can1Tx0Data[1] = data1;
	  can1Tx0Data[0] = data0;
//
//	  can1Tx0Data[7] = 0;
//	  can1Tx0Data[6] = 10;
//	  can1Tx0Data[5] = 13;
//	  can1Tx0Data[4] = 111;
//	  can1Tx0Data[3] = 108;
//	  can1Tx0Data[2] = 108;
//	  can1Tx0Data[1] = 101;
//	  can1Tx0Data[0] = 72;

	  TxMailBox1 = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
	  HAL_CAN_AddTxMessage(&hcan1, &can1TxHeader, &can1Tx0Data[0], &TxMailBox1);
}

/* Receiving CAN Data */
/* Get CAN1 message Through FIFO0 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &can1RxHeader, &can1Rx0Data[0]);
//	can1_rx0_flag = 1;
	printf("1231241\r\n");
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	printf("%d\r\n", dis[0]);
	//CAN_Send(0x10, dis[0] >> 8, dis[0] & 0xFF, dis[1] >> 8, dis[1] & 0xFF, dis[2] >> 8, dis[2] & 0xFF, dis[3] >> 8, dis[3] & 0xFF);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	GetAllData(nof, dis);
	CAN_Send(0x10, dis[0] >> 8, dis[0] & 0xFF, dis[1] >> 8, dis[1] & 0xFF, dis[2] >> 8, dis[2] & 0xFF, dis[3] >> 8, dis[3] & 0xFF);
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
