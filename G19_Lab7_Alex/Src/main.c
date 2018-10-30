
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include "cmsis_os.h"

#include "stm32l475e_iot01_accelero.h"
#include "stm32l475e_iot01_hsensor.h"
#include "stm32l475e_iot01_gyro.h"
#include "stm32l475e_iot01_psensor.h"
#include "stm32l475e_iot01_tsensor.h"
#include "stm32l475e_iot01_magneto.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
osThreadId usartTaskHandle;
osThreadId sensorTaskHandle;
osThreadId userInputTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef handle;

int16_t outputDataArray[3];			// array of integers for accelerometer and magnetic sensor output
float outputDataFloatArray[3];	// array of floats for gyroscope sensor output
float outputDataFloat; 					// floating point variable for temperature, humidity, and pressure sensor outputs
int sensor = 0;									// holds the sensor that is currently being checked

int buttonTimerExpiredFlag = 0;				// when 1, the button has been held for 3 seconds
int sensorTimerExpiredFlag = 0;				// when 1, the sensor can read another value (maintain 5 Hz reading frequency)
int printTimerExpiredFlag = 0;				// not used

uint32_t  buttonExec;						// argument for the timer call back function
osTimerId buttonTimer;					// id for the button timer
uint32_t  sensorExec;						// argument for the timer call back function
osTimerId sensorTimer;					// id for the sensor timer
uint32_t  printExec;						// argument for the timer call back function
osTimerId printTimer;						// id for the print timer

osMutexId mutexID;						// ID for the mutex for the threads

void ButtonTimer_Callback  (void const *arg) {		// timer callback function
	buttonTimerExpiredFlag = 1;
}

int fputc(int ch, FILE *f) {
  while (HAL_OK != HAL_UART_Transmit(&handle, (uint8_t *) &ch, 1, 30000));
  return ch;
}
int fgetc(FILE *f) {
  uint8_t ch = 0;
  while (HAL_OK != HAL_UART_Receive(&handle, (uint8_t *)&ch, 1, 30000));
  return ch;
}

void UART_init() {	
	GPIO_InitTypeDef gpio_init;
	
	handle.Instance = USART1;
	handle.Init.BaudRate = 115200;
  handle.Init.WordLength = UART_WORDLENGTH_8B;
  handle.Init.StopBits = UART_STOPBITS_1;
  handle.Init.Parity = UART_PARITY_NONE;
  handle.Init.Mode = UART_MODE_TX_RX;
  handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  handle.Init.OverSampling = UART_OVERSAMPLING_16;
  handle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  handle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_USART1_CLK_ENABLE();
	
  gpio_init.Pin = GPIO_PIN_6;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOB, &gpio_init);
	
	gpio_init.Pin = GPIO_PIN_7;
	HAL_GPIO_Init(GPIOB, &gpio_init);
	
	HAL_UART_Init(&handle);
}

void GPIO_Init() {
	/** Pinout Configuration
*/
	GPIO_InitTypeDef GPIO_InitStruct;
	
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	
	/*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}	
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void StartUSARTTask(void const * argument);
void StartSensorTask(void const * argument);
void StartUserInputTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  osMutexDef(Mutex1);
	mutexID = osMutexCreate(osMutex(Mutex1));
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  osTimerDef(ButtonTimer, ButtonTimer_Callback);
	buttonTimer = osTimerCreate(osTimer(ButtonTimer), osTimerPeriodic, &buttonExec);
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartUSARTTask, osPriorityNormal, 0, 128);
  usartTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
	
	osThreadDef(sensorTask, StartSensorTask, osPriorityNormal, 0, 128);
  sensorTaskHandle = osThreadCreate(osThread(sensorTask), NULL);
	
	osThreadDef(userInputTask, StartUserInputTask, osPriorityNormal, 0, 128);
  userInputTaskHandle = osThreadCreate(osThread(userInputTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartUSARTTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
	UART_init();
  /* Infinite loop */
  for(;;)
  {
		osDelay(500);			// delay in milliseconds for a reasonable printing frequency
		
		osMutexWait(mutexID, osWaitForever);								// hold mutex so printing doesn't get interrupted
		// print sensor type and data from global variables
		if (sensor == 0) {
			printf("Acceleration: ");
			printf("X = %d, Y = %d, Z = %d\n", outputDataArray[0], outputDataArray[1], outputDataArray[2]);
		} else if (sensor == 1) {
			printf("Gyroscope: ");
			printf("X = %f, Y = %f, Z = %f\n", outputDataFloatArray[0], outputDataFloatArray[1], outputDataFloatArray[2]);
		} else if (sensor == 2) {
			printf("Pressure: %f\n", outputDataFloat);
		} else if (sensor == 3) {
			printf("Magnetometer: ");
			printf("X = %d, Y = %d, Z = %d\n", outputDataArray[0], outputDataArray[1], outputDataArray[2]);
		} else if (sensor == 4) {
			printf("Temperature: %f\n", outputDataFloat);
		} else if (sensor == 5) {
			printf("Humidity: %f\n", outputDataFloat);
		}
		osMutexRelease(mutexID);
  }
  /* USER CODE END 5 */ 
}

/* StartUserInputTask function */
void StartUserInputTask(void const * argument)
{
  /* USER CODE BEGIN */
	GPIO_Init();
	
	int held = 0;
	int loopDelay = 50;
	int oldSensor;
  /* Infinite loop */
  for(;;)
  {
		// wait the length of the loop delay
		osDelay(loopDelay);
		
		// watch for blue button press
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) != GPIO_PIN_SET) {
			if (held == 0) {			// if this is the first notice of the button press (i.e. not held before), change sensors
				sensor++;										// change sensors
				
				if (sensor > 5) {		// rotate back to the first sensor if needed
					sensor = 0;
				}
				
				// osTimerStart(buttonTimer, 3000);		// start the hold timer from 3 seconds
				held = 1;														// indicate that the button is now being held down
			}
			held += loopDelay;
			
			osMutexWait(mutexID, osWaitForever);		// hold mutex during sleep mode so nothing else runs (sleeping!)
			if (held > 3000) {		// if held for more than 3 seconds, suspend sensor readings
				printf("Entering sleep mode.\n");
				
				buttonTimerExpiredFlag = 0;
				oldSensor = sensor;
				sensor = -1;
				
				// turn on the light while suspended
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
				
				while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) != GPIO_PIN_SET) {
					// wait for the button to be released before doing other things
				}
				
				while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET) {
					// until sensor readings are resumed, no need to do anything here. Holding until button pressed again.
				}
				
				// turn off the light
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
			
				// return to the sensor we were on
				sensor = oldSensor;
				
				// send held value back to 0
				held = 0;
				
				printf("Exiting sleep mode.\n");
			}
			osMutexRelease(mutexID);
			
    } else {
			held = 0;
			osTimerStop(buttonTimer);			// stop the timer from counting up. On the next button press, it will reset to 0
		}
  }
  /* USER CODE END 5 */ 
}

/* StartSensorTask function */
void StartSensorTask(void const * argument)
{

  /* USER CODE BEGIN */
	BSP_ACCELERO_Init();
	BSP_GYRO_Init();
	BSP_PSENSOR_Init();
	BSP_MAGNETO_Init();
	BSP_TSENSOR_Init();
	BSP_HSENSOR_Init();
	
  /* Infinite loop */
  for(;;)
  {
		osDelay(200);	// sample every 1/5 of a second (5 Hz)
		
		osMutexWait(mutexID, osWaitForever);					// block other threads from running when entering this section
		if (sensor == 0) {
			/* accelerometer */
			BSP_ACCELERO_AccGetXYZ(outputDataArray);
		} else if (sensor == 1) {
			/* gyroscope */
			BSP_GYRO_GetXYZ(outputDataFloatArray);
		} else if (sensor == 2) {
			/* pressure */
			outputDataFloat = BSP_PSENSOR_ReadPressure();
		} else if (sensor == 3) {
			/* magnetometer */
			BSP_MAGNETO_GetXYZ(outputDataArray);
		} else if (sensor == 4) {
			/* temperature */
			outputDataFloat = BSP_TSENSOR_ReadTemp();
		} else if (sensor == 5) {
			/* humidity */
			outputDataFloat = BSP_HSENSOR_ReadHumidity();
		}
		osMutexRelease(mutexID);
  }
  /* USER CODE END 5 */ 
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM17) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
