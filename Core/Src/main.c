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
#include"i2c-lcd.h"
#include<stdio.h>
#include<stdint.h>
#define CAN_BUFFER_SIZE 10
#define DEBOUNCE_DELAY 100 // Adjust this value as needed
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

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
CAN_TxHeaderTypeDef TxHeader;

uint8_t TxData[8];
uint32_t TxMailbox;



typedef struct {
  CAN_RxHeaderTypeDef header;
  uint8_t data[8];
} CanMessage;

uint8_t mode = 0;
uint32_t lastDebounceTime = 0;



CanMessage canBuffer[CAN_BUFFER_SIZE];
uint8_t bufferIndex = 0;
uint8_t bufferIndexForIds[3] = {0}; // One index for each ID
uint32_t lastCANMessageTime = 0;

// Define a timeout value (in milliseconds) for no CAN messages
#define CAN_TIMEOUT 2000  // Adjust the timeout value as needed



// Function to display "No CAN Messages Received" message on LCD
void displayNoCANMessagesMessage(void)
{

    lcd_send_cmd(0x80 | 0x00);
    lcd_send_string("     TKE MULTI   ");
    lcd_send_cmd(0x80 | 0x40);
    lcd_send_string("    CEEB MONITOR  ");
    lcd_send_cmd(0x80 | 0x14);
    lcd_send_string("No CAN Msg received");
    lcd_send_cmd(0x80 | 0x54);
    lcd_send_string("Version:  2024-01-16");

}

// Function to convert 2's complement binary to decimal
int32_t twosComplementToDecimal(uint8_t b4, uint8_t b5, uint8_t b6, uint8_t b7)
{
    // Combine the bytes into a 32-bit integer
    uint32_t binaryValue = (int32_t)((b7 << 24) | (b6 << 16) | (b5 << 8) | b4);

    // Check if the most significant bit is 1 (indicating a negative number in 2's complement)
    if (binaryValue & 0x80000000)
    {
        //Perform 2's complement conversion
        binaryValue = (~binaryValue) + 1;
        return -binaryValue;  // Convert to signed integer
    }
    else
    {
        return binaryValue;  // Positive number, no conversion needed
    }
}



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (bufferIndex < CAN_BUFFER_SIZE)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &(canBuffer[bufferIndex].header), canBuffer[bufferIndex].data);

        // Check if the current ID is one of the IDs you are interested in (0x112, 0x122, 0x142)
        if (canBuffer[bufferIndex].header.StdId == 0x112 ||
            canBuffer[bufferIndex].header.StdId == 0x122 ||
            canBuffer[bufferIndex].header.StdId == 0x142)
        {
            // Prioritize messages based on cycle time
            uint8_t index = canBuffer[bufferIndex].header.StdId == 0x112 ? 0 :
                            canBuffer[bufferIndex].header.StdId == 0x122 ? 1 : 2;

            bufferIndexForIds[index]++;
        }

        bufferIndex++;
    }
    else
    {
        // Buffer is full, overwrite the oldest message
        bufferIndex = 0;

        // Copy the new message into the buffer
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &(canBuffer[bufferIndex].header), canBuffer[bufferIndex].data);

        // Check if the current ID is one of the IDs you are interested in (0x112, 0x122, 0x142)
        if (canBuffer[bufferIndex].header.StdId == 0x112 ||
            canBuffer[bufferIndex].header.StdId == 0x122 ||
            canBuffer[bufferIndex].header.StdId == 0x142)
        {
            // Prioritize messages based on cycle time
            uint8_t index = canBuffer[bufferIndex].header.StdId == 0x112 ? 0 :
                            canBuffer[bufferIndex].header.StdId == 0x122 ? 1 : 2;

            bufferIndexForIds[index]++;
        }

        bufferIndex++;
    }
    // Update the last CAN message time
    lastCANMessageTime = HAL_GetTick();

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
  if (GPIO_Pin == User_Pushbutton_Pin) {
	  if ((HAL_GetTick() - lastDebounceTime) > DEBOUNCE_DELAY) {
		  mode++;
		  uint8_t TxData[8] = {0};
		  if (mode > 1)
		   mode = 0;
		  if (mode == 1)
		  	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
		  else {
		  	TxData[4] = 0x01;
		  	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
		  }
		  lastDebounceTime = HAL_GetTick(); // Update last debounce time
	  }

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
  MX_CAN1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  TxHeader.DLC = 8;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.StdId = 0x100;


  lcd_init();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  for (uint8_t i = 0; i < CAN_BUFFER_SIZE; i++) {




          if (canBuffer[i].header.StdId == 0x112) {

        	  // Combine 3rd and 4th bits in little-endian format
        	  uint16_t combinedValue = (canBuffer[i].data[3] << 8) | canBuffer[i].data[2];

			  lcd_send_cmd(0x80 | 0x00);
		  	  lcd_send_string("Voltage:");
		  	  lcd_send_float(combinedValue*0.1, 5);
		  	  lcd_send_string("V");

		  }

	      else if (canBuffer[i].header.StdId == 0x142) {


	    	  lcd_send_cmd(0x80 | 0x54);
			  lcd_send_string("MaxCellTemp:");
			  lcd_send_float(canBuffer[i].data[2], 4);
			  lcd_send_string("C   ");
			  lcd_send_cmd(0x80 | 0x14);
			  lcd_send_string("Brick Status:");
			  if (canBuffer[i].data[6] == 0)
				  lcd_send_string("Init  ");
			  else if (canBuffer[i].data[6] == 1)
				  lcd_send_string("Ready ");
			  else if (canBuffer[i].data[6] == 2)
				  lcd_send_string("Wait  ");
			  else if (canBuffer[i].data[6] == 3)
				  lcd_send_string("Active");
			  else if (canBuffer[i].data[6] == 4)
				  lcd_send_string("Error ");
			  else
				  lcd_send_string("?");


			  //lcd_send_integer(canBuffer[i].data[6], 2);
		  }

	      else if (canBuffer[i].header.StdId == 0x122) {

	    	  // Convert 2's complement binary to decimal
	    	  int32_t decimalValue = twosComplementToDecimal(canBuffer[i].data[4], canBuffer[i].data[5], canBuffer[i].data[6], canBuffer[i].data[7]);
	    	  lcd_send_cmd(0x80 | 0x40);
	    	  lcd_send_string("Current:");
	    	  //lcd_send_float(canBuffer[i].data[1], 5);
	    	  lcd_send_float(decimalValue*0.00001, 5);
	    	  lcd_send_string("A  ");


	      }



      }
	  // Check if there are no CAN messages received within the timeout period
	  if ((HAL_GetTick() - lastCANMessageTime) > CAN_TIMEOUT)
	  {

	    	  displayNoCANMessagesMessage();
	    	  HAL_Delay(1000);

	  }


	  HAL_Delay(1000);

  /* USER CODE END 3 */
  }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hcan1.Init.Prescaler = 24;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
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


  CAN_FilterTypeDef canfilterconfig;

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 1;  // which filter bank to use from the assigned ones
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canfilterconfig.FilterIdHigh = (0x112 << 5);
  canfilterconfig.FilterIdLow = 0x0000;
  canfilterconfig.FilterMaskIdHigh = 0x7FF<<5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 20;  // how many filters to assign to the CAN1 (master can)
  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);


  canfilterconfig.FilterBank = 2;  // Use the next available filter bank
  canfilterconfig.FilterIdHigh = (0x122 << 5);
  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);

  canfilterconfig.FilterBank = 3;  // Use the next available filter bank
  canfilterconfig.FilterIdHigh = (0x142 << 5);
  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);


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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : User_Pushbutton_Pin */
  GPIO_InitStruct.Pin = User_Pushbutton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(User_Pushbutton_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
