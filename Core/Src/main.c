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
#include "can.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

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

/* USER CODE BEGIN PV */
volatile uint32_t counter0 = 0;
volatile int16_t curPos0 = 0;
volatile int16_t speed0 = 0;

volatile uint32_t counter1 = 0;
volatile int16_t curPos1 = 0;
volatile int16_t speed1 = 0;

volatile uint16_t t0 = 0;
volatile uint16_t t1 = 0;

volatile uint8_t can2_rx0_flag = 0;

uint16_t TOF0  = 0x52;
uint16_t TOF1  = 0x54;
uint16_t TOF2  = 0x56;
uint16_t TOF3  = 0x58;
uint16_t TOF4  = 0x5A;
uint16_t TOF5  = 0x5C;
uint16_t TOF6  = 0x5E;
uint16_t TOF7  = 0x60;
uint16_t TOF8  = 0x62;
uint16_t TOF9  = 0x64;
uint16_t TOF10 = 0x66;
uint16_t TOF11 = 0x68;
uint16_t TOF12 = 0x6A;
uint16_t TOF13 = 0x6C;
uint16_t TOF14 = 0x6E;

VL53L1X_Result_t tof0_result;
VL53L1X_Result_t tof1_result;
VL53L1X_Result_t tof2_result;
VL53L1X_Result_t tof3_result;
VL53L1X_Result_t tof4_result;
VL53L1X_Result_t tof5_result;
VL53L1X_Result_t tof6_result;
VL53L1X_Result_t tof7_result;
VL53L1X_Result_t tof8_result;
VL53L1X_Result_t tof9_result;
VL53L1X_Result_t tof10_result;
VL53L1X_Result_t tof11_result;
VL53L1X_Result_t tof12_result;
VL53L1X_Result_t tof13_result;
VL53L1X_Result_t tof14_result;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void setLeftMotorSpeed(int16_t speed) {
	 uint32_t ccr = (uint32_t) speed * 4 * 1000;
	 if (ccr > TIM1->ARR) return;
	 TIM1->CCR1 = ccr;
	  //HAL_Delay(50);
}

void setRightMotorSpeed(int16_t speed) {
	  uint32_t ccr = (uint32_t) speed * 4 * 1000;
	  if (ccr > TIM1->ARR) return;
	  TIM1->CCR2 = ccr;
	  //HAL_Delay(50);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */

	if (htim == &htim2) {
		counter0 = __HAL_TIM_GET_COUNTER(htim);

		curPos0 = (int16_t) counter0 / 4;
	}
	else if (htim == &htim3) {
		counter1 = __HAL_TIM_GET_COUNTER(htim);

		curPos1 = (int16_t) counter1 / 4;
	}

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_IC_CaptureCallback could be implemented in the user file
   */
}

int _write(int file, char* p, int len) {
	HAL_UART_Transmit(&huart2, p, len, 16);
	return len;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch (GPIO_Pin) {
		case GPIO_PIN_0:
			VL53L1X_GetResult(TOF0, &tof0_result);
			VL53L1X_ClearInterrupt(TOF0);
			break;
		case GPIO_PIN_1:
			VL53L1X_GetResult(TOF1, &tof1_result);
			VL53L1X_ClearInterrupt(TOF1);
			break;
		case GPIO_PIN_2:
			VL53L1X_GetResult(TOF2, &tof2_result);
			VL53L1X_ClearInterrupt(TOF2);
			break;
		case GPIO_PIN_3:
			VL53L1X_GetResult(TOF3, &tof3_result);
			VL53L1X_ClearInterrupt(TOF3);
			break;
		case GPIO_PIN_4:
			VL53L1X_GetResult(TOF4, &tof4_result);
			VL53L1X_ClearInterrupt(TOF4);
			break;
		case GPIO_PIN_5:
			VL53L1X_GetResult(TOF5, &tof5_result);
			VL53L1X_ClearInterrupt(TOF5);
			break;
		case GPIO_PIN_6:
			VL53L1X_GetResult(TOF6, &tof6_result);
			VL53L1X_ClearInterrupt(TOF6);
			break;
		case GPIO_PIN_7:
			VL53L1X_GetResult(TOF7, &tof7_result);
			VL53L1X_ClearInterrupt(TOF7);
			break;
		case GPIO_PIN_8:
			VL53L1X_GetResult(TOF8, &tof8_result);
			VL53L1X_ClearInterrupt(TOF8);
			break;
		case GPIO_PIN_9:
			VL53L1X_GetResult(TOF9, &tof9_result);
			VL53L1X_ClearInterrupt(TOF9);
			break;
		case GPIO_PIN_10:
			VL53L1X_GetResult(TOF10, &tof10_result);
			VL53L1X_ClearInterrupt(TOF10);
			break;
		case GPIO_PIN_11:
			VL53L1X_GetResult(TOF11, &tof11_result);
			VL53L1X_ClearInterrupt(TOF11);
			break;
		case GPIO_PIN_12:
			VL53L1X_GetResult(TOF12, &tof12_result);
			VL53L1X_ClearInterrupt(TOF12);
			break;
		case GPIO_PIN_13:
			VL53L1X_GetResult(TOF13, &tof13_result);
			VL53L1X_ClearInterrupt(TOF13);
			break;
		case GPIO_PIN_14:
			VL53L1X_GetResult(TOF14, &tof14_result);
			VL53L1X_ClearInterrupt(TOF14);
			break;
		default:
			break;
	}
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
  MX_TIM1_Init();
  MX_CAN2_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();

  uint8_t all_booted = 0;
  while (!all_booted) {
	  uint8_t boot_state[15] = {0};
	  VL53L1X_BootState(TOF0, &boot_state[0]);
	  VL53L1X_BootState(TOF1, &boot_state[1]);
	  VL53L1X_BootState(TOF2, &boot_state[2]);
	  VL53L1X_BootState(TOF3, &boot_state[3]);
	  VL53L1X_BootState(TOF4, &boot_state[4]);
	  VL53L1X_BootState(TOF5, &boot_state[5]);
	  VL53L1X_BootState(TOF6, &boot_state[6]);
	  VL53L1X_BootState(TOF7, &boot_state[7]);
	  VL53L1X_BootState(TOF8, &boot_state[8]);
	  VL53L1X_BootState(TOF9, &boot_state[9]);
	  VL53L1X_BootState(TOF10, &boot_state[10]);
	  VL53L1X_BootState(TOF11, &boot_state[11]);
	  VL53L1X_BootState(TOF12, &boot_state[12]);
	  VL53L1X_BootState(TOF13, &boot_state[13]);
	  VL53L1X_BootState(TOF14, &boot_state[14]);

	  all_booted = 1;
	  for (uint8_t i = 0; i != 15; ++i) {
		  all_booted *= boot_state[i];
	  }
  }

  VL53L1X_SensorInit(TOF0);
  /*
  VL53L1X_SensorInit(TOF1);
  VL53L1X_SensorInit(TOF2);
  VL53L1X_SensorInit(TOF3);
  VL53L1X_SensorInit(TOF4);
  VL53L1X_SensorInit(TOF5);
  VL53L1X_SensorInit(TOF6);
  VL53L1X_SensorInit(TOF7);
  VL53L1X_SensorInit(TOF8);
  VL53L1X_SensorInit(TOF9);
  VL53L1X_SensorInit(TOF10);
  VL53L1X_SensorInit(TOF11);
  VL53L1X_SensorInit(TOF12);
  VL53L1X_SensorInit(TOF13);
  VL53L1X_SensorInit(TOF14);
   */
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);

  VL53L1X_StartRanging(TOF0);
  /*
  VL53L1X_StartRanging(TOF1);
  VL53L1X_StartRanging(TOF2);
  VL53L1X_StartRanging(TOF3);
  VL53L1X_StartRanging(TOF4);
  VL53L1X_StartRanging(TOF5);
  VL53L1X_StartRanging(TOF6);
  VL53L1X_StartRanging(TOF7);
  VL53L1X_StartRanging(TOF8);
  VL53L1X_StartRanging(TOF9);
  VL53L1X_StartRanging(TOF10);
  VL53L1X_StartRanging(TOF11);
  VL53L1X_StartRanging(TOF12);
  VL53L1X_StartRanging(TOF13);
  VL53L1X_StartRanging(TOF14);
   */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  canFilter1.FilterMaskIdHigh = 0x7F3 << 5;
  canFilter1.FilterIdHigh = 0x106 << 5;
  canFilter1.FilterMaskIdLow = 0x753 << 5;
  canFilter1.FilterIdLow = 0x106 << 5;
  canFilter1.FilterMode = CAN_FILTERMODE_IDMASK;
  canFilter1.FilterScale = CAN_FILTERSCALE_16BIT;
  canFilter1.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canFilter1.FilterBank = 0;
  canFilter1.FilterActivation = ENABLE;

  HAL_CAN_ConfigFilter(&hcan2, &canFilter1);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_Start(&hcan2);

  setLeftMotorSpeed(-49);
  setRightMotorSpeed(-19);

  while (1)
  {
	  if (can2_rx0_flag) {
		  can2_rx0_flag = 0;

		  // receive and do something..
	  }

	  canTxHeader.StdId = 0x102;
	  canTxHeader.RTR = CAN_RTR_DATA;
	  canTxHeader.IDE = CAN_ID_STD;
	  canTxHeader.DLC = 8;

	  TxMailBox = HAL_CAN_GetTxMailboxesFreeLevel(&hcan2);
	  can2Tx0Data[0] = (speed0 & 0xFF00) >> 8;
	  can2Tx0Data[1] = (speed0 & 0x00FF);
	  can2Tx0Data[2] = (speed1 & 0xFF00) >> 8;
	  can2Tx0Data[3] = (speed1 & 0x00FF);
 	  HAL_CAN_AddTxMessage(&hcan2, &canTxHeader, &can2Tx0Data[0], &TxMailBox);

	  printf("%d %d\n", speed0, curPos0);
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
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
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
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
	if (hcan->Instance == CAN2) {
		HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &canRxHeader, &can2Rx0Data[0]);
		can2_rx0_flag = 1;
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
