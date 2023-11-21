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

#define ACC 1

/* USER CODE END PFP */

unsigned char tx_Data[1];
unsigned char rx_Data[1];
uint32_t rx_ID;
uint8_t rx_DLC;

uint32_t tx_ID = 0x030;
uint8_t tx_DLC = 1;
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// Function to transmit a standard ID data frame
void CAN_tx(uint32_t ID, uint32_t DLC, uint8_t *p_payload, uint8_t polling_on) {
	// Configure the transmission header
	CAN_TxHeaderTypeDef pheader;
	uint32_t TxMailbox;

	if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan)) {
		pheader.DLC = DLC;
		pheader.IDE = CAN_ID_STD;
		pheader.RTR = CAN_RTR_DATA;
		pheader.StdId = ID;

		// Add the message to the CAN transmit queue
		if (HAL_CAN_AddTxMessage(&hcan, &pheader, p_payload, &TxMailbox) != HAL_OK) {
			Error_Handler();
		}
	}

	// If polling is enabled, wait until the frame is done transmitting
	if (polling_on) {
		while (HAL_CAN_IsTxMessagePending(&hcan, TxMailbox));
	}
}

// Function to configure CAN reception filter
void CAN_rx_FilterConfig(uint16_t stdID, uint16_t stdMASK) {
	// Configure the CAN filter
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

	// Apply the filter configuration
	if (HAL_CAN_ConfigFilter(&hcan, &filterConfig) != HAL_OK) {
		Error_Handler();
	}
}

// Function to receive CAN data
void CAN_rx(uint32_t *ID, uint8_t *DLC, uint8_t *p_payload, uint8_t polling_on) {
	// If polling is enabled, wait until at least one message is received
	if (polling_on) {
		while (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) == 0);
	}

	// Receive the message from the CAN receive queue
	CAN_RxHeaderTypeDef pHeader;
	if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &pHeader, p_payload) != HAL_OK) {
		Error_Handler();
	}

	// Extract message details
	*DLC = pHeader.DLC;
	*ID = pHeader.StdId;
}




/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */

// IRQ callback for CAN transmission completion in Mailbox 0
// Note: This callback is invoked when the transmission in Mailbox 0 is complete.
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) {
	// For an ECU (1) that is done transmitting, no additional action is required.
	// Add specific handling if needed for other cases.
}

// IRQ callback for CAN FIFO 0 message pending
// Note: This callback is invoked when a message is pending in receive FIFO 0.
// It triggers the reception of speed data using the CAN_rx function.
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	// Receive speed data using the CAN_rx function.
	CAN_rx(&rx_ID, &rx_DLC, rx_Data, 0);
}

// IRQ callback for CAN errors
// Note: This callback is invoked when a CAN error occurs.
// In this example, there is a conditional check for a specific transmission error (TX_TERR0).
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
	// Check if the error is related to transmission error TX_TERR0
	if ((hcan->ErrorCode & HAL_CAN_ERROR_TX_TERR0) == HAL_CAN_ERROR_TX_TERR0) {
		// Handle the specific transmission error TX_TERR0.
	}
	// Add additional checks and error handling for other error types if required.
}






int main(void)
{
	/* USER CODE BEGIN 1 */
	// Any user-specific code or variable declarations can be placed here.
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface, and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
	// Additional user initialization code can be added here.
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */
	// Additional system initialization code can be added here.
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_CAN_Init();

	// Activate interrupts:
	// - Transmission Mailbox Empty: Indicates that a message has been sent successfully.
	// - FIFO Pending - Message to be Received: Indicates that a message is pending in the receive FIFO.
	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_TX_MAILBOX_EMPTY || CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
		// If activation fails, handle the error.
		Error_Handler();
	}

	// Configure the filter
	CAN_rx_FilterConfig(0x3AB, 0x7FF);

	// Start the bxCAN to begin transmitting
	if (HAL_CAN_Start(&hcan) != HAL_OK) {
		// If starting fails, handle the error.
		Error_Handler();
	}

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */


	tx_Data[0] = ACC;
	while (1)
	{
		/* USER CODE END WHILE */

		HAL_Delay(1000);  // Pause for 1000 milliseconds

		// Transmit CAN message

		CAN_tx(tx_ID, tx_DLC, tx_Data, 0);
		tx_Data[0] ^= 1;  // Toggle the data for the next transmission

		/* USER CODE BEGIN 3 */
		// Any additional user code or processing can be added here.
		/* USER CODE END 3 */
	}
	/* USER CODE BEGIN 3 */
	// Additional user code or processing after the infinite loop can be added here.
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
	hcan.Init.Mode = CAN_MODE_NORMAL;
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
