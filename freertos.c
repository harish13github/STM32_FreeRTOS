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
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "string.h"
#include "stdio.h"

/* USER CODE END Includes */
typedef struct
{
	int16_t Rx_x;
	int16_t Rx_y;
	int16_t Rx_z;
}Sensordata;

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

UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */
int8_t Return[2], TxBuf[2];    // return buff is to recieved the data and tx buff is to stored and transmit data
//int16_t Rx_x, Rx_y,Rx_z;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */
#define DWT_CYCCNT ((volatile uint32_t *) 0xE0001000)
TaskHandle_t task1;
TaskHandle_t task2;
TaskHandle_t task3;
//QueueHandle_t q1;
QueueHandle_t q1;
QueueHandle_t q2;
void Task1 (void *a)
{
	//QueueHandle_t q1= ( *(QueueHandle_t *)a);
//	QueueHandle_t q1= *((QueueHandle_t *)a);
	for(;;){
		Sensordata value;
		 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, RESET);
			 			  TxBuf[0]=0x29|0x80;  // read
			 			 // TxBuf[0]=0x2B|0x80|0x40;  // read
			 			  HAL_SPI_Transmit(&hspi1, TxBuf, 1, 50);
			 			  HAL_SPI_Receive(&hspi1, Return, 1, 50);
			 			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, SET);
			 			// Rx_x= (Return[1]<<8) | Return[0];
			 			 value.Rx_x= (Return[1]<<8) | Return[0];

			 			  // reading the value of the Y-Register
			 			 	  // OUTX address 2Ah-2Bh
			 			 	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, RESET);
			 			 	  TxBuf[0]=0x2B|0x80;  // read
			 			 	 // TxBuf[0]=0x2B|0x80|0x40;  // read
			 			 	  HAL_SPI_Transmit(&hspi1, TxBuf, 1, 50);
			 			 	  HAL_SPI_Receive(&hspi1, Return, 1, 50);
			 			 	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, SET);
			 			 //	Rx_y= (Return[1]<<8) | Return[0];
			 				  value.Rx_y= (Return[1]<<8) | Return[0];

			 			 	 // reading the value of the Z-Register
			 			 		  // OUTX address 2ch-2Dh
			 			 		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, RESET);
			 			 		  TxBuf[0]=0x2D|0x80;  // read
			 			 		 // TxBuf[0]=0x2B|0x80|0x40;  // read
			 			 		  HAL_SPI_Transmit(&hspi1, TxBuf, 1, 50);
			 			 		  HAL_SPI_Receive(&hspi1, Return, 1, 50);
			 			 		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, SET);
			 			 	//	Rx_z= (Return[1]<<8) | Return[0];
			 			 		 value.Rx_z= (Return[1]<<8) | Return[0];
		xQueueSend(q1,&value,portMAX_DELAY);
  SEGGER_SYSVIEW_PrintfHost(&value);
  vTaskDelay(pdMS_TO_TICKS(500));
		//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13 | GPIO_PIN_14);

	}
}

void Task2 (void *a)
{
//	QueueHandle_t q1= *((QueueHandle_t *)a);
//	QueueHandle_t q2= *((QueueHandle_t *)a);
	//QueueHandle_t q1 =q[0];
//	QueueHandle_t q2 =q[1];
	for(;;){
		Sensordata Rvalue;
		//if( xQueueReceive( xQueue, &xMessage, portMAX_DELAY )
		xQueueReceive(q1, &Rvalue,portMAX_DELAY);
		 SEGGER_SYSVIEW_PrintfHost(&Rvalue);
		char xyzstring[50];


		sprintf(xyzstring, "X value is=%d, Y value is =%d, Z value is =%d\r\n", Rvalue.Rx_x,Rvalue.Rx_y,Rvalue.Rx_z);

		xQueueSend(q2, xyzstring, portMAX_DELAY);
		 SEGGER_SYSVIEW_PrintfHost(xyzstring);

	/*	if(Rvalue.Rx_x) {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12,SET);
			vTaskDelay(100);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12,RESET);
			vTaskDelay(100);
		}
		else if (Rvalue.Rx_y) {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13,SET);
			vTaskDelay(100);
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13,RESET);
						vTaskDelay(100);
		}
		else if (Rvalue.Rx_z) {
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14,SET);
					vTaskDelay(100);
								HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14,RESET);
								vTaskDelay(100);
		}*/
		vTaskDelay( pdMS_TO_TICKS(500));
	}
}
void Task3 (void *a)
{
//	QueueHandle_t q2= *((QueueHandle_t *)a);
	//QueueHandle_t *q= (QueueHandle_t *)a;
	for(;;) {
		char strR[50];

		xQueueReceive(q2, strR,portMAX_DELAY);
		SEGGER_SYSVIEW_PrintfHost(&strR);
		HAL_UART_Transmit(&huart5, (void *)strR, strlen(strR), 100);
		SEGGER_SYSVIEW_PrintfHost(strR);
 vTaskDelay(pdMS_TO_TICKS(500));

	}
}
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
//	static QueueHandle_t q1;
//	static QueueHandle_t q2;

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
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  *DWT_CYCCNT = *DWT_CYCCNT | (1 << 0);
    SEGGER_SYSVIEW_Conf();
    SEGGER_SYSVIEW_Start();

  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, RESET);  //pull chip select pin low
   TxBuf[0]=0x20; // Address of Register
   TxBuf[1]=0x37; // Data to be Filled
   HAL_SPI_Transmit(&hspi1, TxBuf, 2, 10);  //  handel of SPI, name of buffer , number of bytes, timeout
   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, SET);  //pull chip select pin High

    q1=xQueueCreate(10,sizeof(Sensordata));
    q2=xQueueCreate(10,sizeof(char[50]));

   // QueueHandle_t q={q1,q2};

//   	 xTaskCreate(Task1, "task1", 200, (void *)&q1, 3,NULL);
// 	 xTaskCreate(Task2, "task2", 200,  (void *)&q1, 2, NULL);
//	 xTaskCreate(Task3, "task3", 200, (void *)&q2, 1, NULL);

    xTaskCreate(Task1, "task1", 200,NULL, 3,NULL);
     	 xTaskCreate(Task2, "task2", 200, NULL, 2, NULL);
    	 xTaskCreate(Task3, "task3", 200,NULL, 1, NULL);
  //xTaskCreate(Task4, "task4", 200, NULL, 4, &task4);
 	 vTaskStartScheduler();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 6, 0);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM10 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM10) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
