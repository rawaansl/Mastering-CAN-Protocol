/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */


// function to transmit with standard ID
// data frame

void CAN_tx(uint32_t ID, uint32_t DLC, uint8_t *p_payload){

	//	  (#) To manage messages transmission, the following Tx control functions
	//	          can be used:
	//	            (++) HAL_CAN_AddTxMessage() to request transmission of a new
	//	                 message.
	//	            (++) HAL_CAN_AbortTxRequest() to abort transmission of a pending
	//	                 message.
	//	            (++) HAL_CAN_GetTxMailboxesFreeLevel() to get the number of free Tx
	//	                 mailboxes.
	//	            (++) HAL_CAN_IsTxMessagePending() to check if a message is pending
	//	                 in a Tx mailbox.
	//	            (++) HAL_CAN_GetTxTimestamp() to get the timestamp of Tx message
	//	                 sent, if time triggered communication mode is enabled.
	uint32_t TxMailbox;
	CAN_TxHeaderTypeDef pheader;
	if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan)){

		pheader.DLC = DLC;
		pheader.IDE = CAN_ID_STD;
		pheader.RTR = CAN_RTR_DATA;
		pheader.StdId = ID;
		if(HAL_CAN_AddTxMessage(&hcan, &pheader , p_payload,  &TxMailbox) != HAL_OK){
			Error_Handler();
		}
	}
	while (HAL_CAN_IsTxMessagePending(&hcan, TxMailbox));    // wait till frame is done transmitting
}


void CAN_rx_FilterConfig(uint16_t stdID, uint16_t stdMASK){


	CAN_FilterTypeDef filterConfig;
	filterConfig.FilterActivation = CAN_FILTER_ENABLE;
	filterConfig.FilterBank = 0;
	filterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	filterConfig.FilterIdHigh = stdID << 5;
	filterConfig.FilterMaskIdHigh = stdMASK << 5;
	filterConfig.FilterIdLow = 0x0000;
	filterConfig.FilterMaskIdLow = 0x0000;
	filterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	filterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

	/**
	 * @brief  Configures the CAN reception filter according to the specified
	 *         parameters in the CAN_FilterInitStruct.
	 * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
	 *         the configuration information for the specified CAN.
	 * @param  sFilterConfig pointer to a CAN_FilterTypeDef structure that
	 *         contains the filter configuration information.
	 * @retval None
	 **/

	if(HAL_CAN_ConfigFilter(&hcan, &filterConfig) != HAL_OK){
		Error_Handler();
	}
}

void CAN_rx(uint32_t *ID, uint8_t *DLC, uint8_t *p_payload){


	//    *** Polling mode operation ***
	//    ==============================
	//  [..]
	//    (#) Reception:
	//          (++) Monitor reception of message using HAL_CAN_GetRxFifoFillLevel()
	//               until at least one message is received.
	//          (++) Then get the message using HAL_CAN_GetRxMessage().

	while(HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) == 0);
	CAN_RxHeaderTypeDef pHeader;
	if(HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &pHeader, p_payload) != HAL_OK){
		Error_Handler();
	}
	*DLC = pHeader.DLC;
	*ID = pHeader.StdId;
}


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
	MX_CAN_Init();


	// Start the bxCAN to start transmitting
	if(HAL_CAN_Start(&hcan) != (HAL_OK)){
		Error_Handler();
	}

	// configure the filter
	CAN_rx_FilterConfig(0x3FF, 0x7FF);




	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	unsigned char tx_Data[8] = {'C', 'A', 'N', ' ', 'N', 'O', ':', ' '};
	uint8_t frameno = 0;
	unsigned char rx_Data[8];
	uint32_t rx_ID;
	uint8_t rx_DLC;
	while (1)
	{
		/* USER CODE END WHILE */
		tx_Data[7] = frameno++;
		CAN_tx(0x3FF, 8, tx_Data);

		CAN_rx(&rx_ID, &rx_DLC, rx_Data);

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

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN_Init(void)
{

	/* USER CODE BEGIN CAN_Init 0 */

	/* USER CODE END CAN_Init 0 */

	/* USER CODE BEGIN CAN_Init 1 */

	/* USER CODE END CAN_Init 1 */
	hcan.Instance = CAN1;
	hcan.Init.Prescaler = 1;
	hcan.Init.Mode = CAN_MODE_LOOPBACK;
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_6TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = DISABLE;
	hcan.Init.AutoWakeUp = DISABLE;
	hcan.Init.AutoRetransmission = ENABLE;
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN CAN_Init 2 */

	/* USER CODE END CAN_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
