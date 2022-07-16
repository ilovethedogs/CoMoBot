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

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//CAN1
CAN_FilterTypeDef canFilter1;
CAN_RxHeaderTypeDef can1RxHeader;
CAN_TxHeaderTypeDef can1TxHeader;

// Related to CAN commuication
uint8_t can1Rx0Data[8];
uint32_t TxMailBox1;
uint8_t can1Tx0Data[8];
volatile uint8_t can1_rx0_flag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void CanSendMssg(void);
void CanRecvMssg(void);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);
void CAN_Filter_Init(void);
void CAN_Send(uint8_t ID, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7);

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
  MX_CAN1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  CAN_Filter_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t outer[6] = {79, 85, 84, 13, 10, 0};
  while (1)
  {
	  //CanSendMssg();
	  //CanRecvMssg();
	  //HAL_UART_Transmit(&huart2, outer, 6, 10);
	  //HAL_Delay(1000);
	  CAN_Send(0x10, 0, 2, 3, 4, 5, 6, 7, 0);

	  if (can1_rx0_flag == 1) {
		  can1_rx0_flag = 0;
		  HAL_UART_Transmit(&huart2, outer, 6, 10);
	  }

	  HAL_Delay(100);
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
  hcan1.Init.Prescaler = 18;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
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
  /*
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }
  */
  /* USER CODE END CAN1_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void CAN_Filter_Init(void)
{
	//Set the CAN Filter which has Mask Ids
	//CAN Filter1
	//Receiving CAN data via 0x102~0x10E
	canFilter1.FilterMaskIdHigh = 0x7F3 << 5; // Shift 5 bit
	canFilter1.FilterIdHigh = 0x106 << 5;
	canFilter1.FilterMaskIdLow = 0x7F3 << 5; // Shift 5 bit
	canFilter1.FilterIdLow = 0x106 << 5;
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

	  can1Tx0Data[7] = 21;
	  can1Tx0Data[6] = 56;
	  can1Tx0Data[5] = 54;
	  can1Tx0Data[4] = 27;
	  can1Tx0Data[3] = 89;
	  can1Tx0Data[2] = 211;
	  can1Tx0Data[1] = 243;
	  can1Tx0Data[0] = 23;

	  TxMailBox1 = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
	  HAL_CAN_AddTxMessage(&hcan1, &can1TxHeader, &can1Tx0Data[0], &TxMailBox1);
}

/* Receiving CAN Data */
/* Get CAN1 message Through FIFO0 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance == CAN1)
	{
		HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &can1RxHeader, &can1Rx0Data[0]);

		can1_rx0_flag = 1;
	}

}
/*
void CanSendMssg(void) {
	  TxHeader.StdId = 0x11;
	  TxHeader.RTR = CAN_RTR_DATA;
	  TxHeader.IDE = CAN_ID_STD;
	  TxHeader.DLC = 2;
	  TxHeader.TransmitGlobalTime = DISABLE;
	  TxData[0] = 0xCA;
	  TxData[1] = 0xFE;

	  uint8_t trysend[10] = {84, 82, 89, 83, 69, 78, 68, 13, 10, 0};
	  HAL_UART_Transmit(&huart2, trysend, 10, 10);

	  uint8_t txbox[7] = {84, 88, 66, 88, 13, 10, 0};
	  if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
		  HAL_UART_Transmit(&huart2, txbox, 7, 10);
		  return;
	  }

	  uint8_t sendfail[11] = {83, 69, 78, 68, 70, 65, 73, 76, 13, 10, 0};
	  if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	  {

		HAL_UART_Transmit(&huart2, sendfail, 11, 10);
	  }

	  //return HAL_OK;
}

void CanRecvMssg(void) {
	uint8_t aa[7] = {70, 70, 70, 70, 13, 10, 0};
	if(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) == 0)
	{
		HAL_UART_Transmit(&huart2, aa, 7, 10);
	}

	uint8_t recvfail[11] = {82, 69, 67, 86, 70, 65, 73, 76, 13, 10, 0};
	if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
	{

		HAL_UART_Transmit(&huart2, recvfail, 11, 10);
	}

	if((RxHeader.StdId != 0x11)                     ||
	   (RxHeader.RTR != CAN_RTR_DATA)               ||
	   (RxHeader.IDE != CAN_ID_STD)                 ||
	   (RxHeader.DLC != 2))
	{

		//return HAL_ERROR;
	}

	HAL_UART_Transmit(&huart2, RxData, 2, 10);

	//return HAL_OK;
}
*/
/*
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CanRecvMssg();
}
*/
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
