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
#include "stm32f7xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "stm32f7xx_it.h"
#include "package.h"

#define BLINKER_ID							0x01
#define R_BLINKER_SUB_ID				0x01
#define L_BLINKER_SUB_ID				0x02
#define LIGHTS_ID								0x02
#define LOW_BEAM_SUB_ID					0x01
#define HIGH_BEAM_SUB_ID				0x02
#define STOP_LIGHT_ID						0x03
#define STOP_LIGHT_SUB_ID				0x01
#define INTERIOR_LIGHT_ID				0x04
#define INTERIOR_LIGHT_SUB_ID		0x01
#define WIPER_ID								0x05
#define WIPER_SUB_ID						0x01
#define DOOR_CLOSED_ID					0x06
#define DOOR_LOCKED_ID					0x07
#define RF_DOOR_SUB_ID					0x01
#define RR_DOOR_SUB_ID					0x02
#define LF_DOOR_SUB_ID					0x03
#define LR_DOOR_SUB_ID					0x04

#define KEY_MASK								0x01
#define I2C_ADDRESS							0x1C


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart3;

osThreadId defaultTaskHandle;
osThreadId myTaskLightLowHandle;
osThreadId myTaskBlinkerRHandle;
osThreadId myTaskStopLightHandle;
osThreadId myTaskInteriorHandle;
osThreadId myTaskProximityHandle;
osThreadId myTaskWiperHandle;
osThreadId myTaskBlinkerLHandle;
osThreadId myTaskLightHighHandle;
osThreadId myTaskClosedRFHandle;
osThreadId myTaskClosedRRHandle;
osThreadId myTaskClosedLFHandle;
osThreadId myTaskClosedLRHandle;
osThreadId myTaskLockedRFHandle;
osThreadId myTaskLockedRRHandle;
osThreadId myTaskLockedLFHandle;
osThreadId myTaskLockedLRHandle;
osMutexId myMutex01Handle;
osMutexId myMutex02Handle;
osMutexId myMutex03Handle;
osMutexId myMutex04Handle;
osMutexId myMutex05Handle;
osMutexId myMutex06Handle;
osMutexId myMutex07Handle;
osMutexId myMutex08Handle;
osMutexId myMutex09Handle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

int16_t button=0;
uint8_t *dataField;
static uint8_t flag = 0;
static uint8_t wiperFlag = 0;

uint8_t data[2];
uint8_t analogData[24];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM11_Init(void);
void StartDefaultTask(void const * argument);
void StartTaskLightsLow(void const * argument);
void StartTaskBlinkersRight(void const * argument);
void StartTaskStopLights(void const * argument);
void StartTaskInterior(void const * argument);
void StartTaskProximity(void const * argument);
void StartTaskWiper(void const * argument);
void StartTaskBlinkersLeft(void const * argument);
void StartTaskLightsHigh(void const * argument);
void StartTaskClosedRF(void const * argument);
void StartTaskClosedRR(void const * argument);
void StartTaskClosedLF(void const * argument);
void StartTaskClosedLR(void const * argument);
void StartTaskLockedRF(void const * argument);
void StartTaskLockedRR(void const * argument);
void StartTaskLockedLF(void const * argument);
void StartTaskLockedLR(void const * argument);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
                                
                                
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void setPWM(TIM_HandleTypeDef, uint32_t, uint16_t, uint16_t);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
HAL_StatusTypeDef status;
uint8_t Test2[5];
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t testData[1] = {25};
	uint8_t pulseScale[1] = {0x21};
	uint8_t removeAutoCalibrate[1] = {0};

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
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_USART3_UART_Init();
  MX_UART4_Init();
  MX_TIM9_Init();
  MX_TIM3_Init();
  MX_TIM12_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */

	HAL_I2C_Mem_Write(&hi2c2, I2C_ADDRESS << 1, 6, I2C_MEMADD_SIZE_8BIT, testData, 1, 100);
	HAL_I2C_Mem_Write(&hi2c2, I2C_ADDRESS << 1, 8, I2C_MEMADD_SIZE_8BIT, testData, 1, 100);
	//HAL_I2C_Mem_Write(&hi2c2, I2C_ADDRESS << 1, 11, I2C_MEMADD_SIZE_8BIT, testData, 1, 100);
	//HAL_I2C_Mem_Write(&hi2c2, I2C_ADDRESS << 1, 12, I2C_MEMADD_SIZE_8BIT, removeAutoCalibrate, 1, 100);
	
	HAL_I2C_Mem_Write(&hi2c2, I2C_ADDRESS << 1, 40, I2C_MEMADD_SIZE_8BIT, pulseScale, 1, 100);
	HAL_I2C_Mem_Write(&hi2c2, I2C_ADDRESS << 1, 44, I2C_MEMADD_SIZE_8BIT, pulseScale, 1, 100);
	HAL_I2C_Mem_Write(&hi2c2, I2C_ADDRESS << 1, 45, I2C_MEMADD_SIZE_8BIT, pulseScale, 1, 100);
	HAL_I2C_Mem_Write(&hi2c2, I2C_ADDRESS << 1, 49, I2C_MEMADD_SIZE_8BIT, pulseScale, 1, 100);
	HAL_I2C_Mem_Write(&hi2c2, I2C_ADDRESS << 1, 50, I2C_MEMADD_SIZE_8BIT, pulseScale, 1, 100);
	HAL_I2C_Mem_Write(&hi2c2, I2C_ADDRESS << 1, 51, I2C_MEMADD_SIZE_8BIT, pulseScale, 1, 100);
	
  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of myMutex01 */
  osMutexDef(myMutex01);
  myMutex01Handle = osMutexCreate(osMutex(myMutex01));

  /* definition and creation of myMutex02 */
  osMutexDef(myMutex02);
  myMutex02Handle = osMutexCreate(osMutex(myMutex02));

  /* definition and creation of myMutex03 */
  osMutexDef(myMutex03);
  myMutex03Handle = osMutexCreate(osMutex(myMutex03));

  /* definition and creation of myMutex04 */
  osMutexDef(myMutex04);
  myMutex04Handle = osMutexCreate(osMutex(myMutex04));

  /* definition and creation of myMutex05 */
  osMutexDef(myMutex05);
  myMutex05Handle = osMutexCreate(osMutex(myMutex05));

  /* definition and creation of myMutex06 */
  osMutexDef(myMutex06);
  myMutex06Handle = osMutexCreate(osMutex(myMutex06));

  /* definition and creation of myMutex07 */
  osMutexDef(myMutex07);
  myMutex07Handle = osMutexCreate(osMutex(myMutex07));

  /* definition and creation of myMutex08 */
  osMutexDef(myMutex08);
  myMutex08Handle = osMutexCreate(osMutex(myMutex08));

  /* definition and creation of myMutex09 */
  osMutexDef(myMutex09);
  myMutex09Handle = osMutexCreate(osMutex(myMutex09));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityAboveNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTaskLightLow */
  osThreadDef(myTaskLightLow, StartTaskLightsLow, osPriorityNormal, 0, 128);
  myTaskLightLowHandle = osThreadCreate(osThread(myTaskLightLow), NULL);

  /* definition and creation of myTaskBlinkerR */
  osThreadDef(myTaskBlinkerR, StartTaskBlinkersRight, osPriorityNormal, 0, 128);
  myTaskBlinkerRHandle = osThreadCreate(osThread(myTaskBlinkerR), NULL);

  /* definition and creation of myTaskStopLight */
  osThreadDef(myTaskStopLight, StartTaskStopLights, osPriorityNormal, 0, 128);
  myTaskStopLightHandle = osThreadCreate(osThread(myTaskStopLight), NULL);

  /* definition and creation of myTaskInterior */
  osThreadDef(myTaskInterior, StartTaskInterior, osPriorityNormal, 0, 128);
  myTaskInteriorHandle = osThreadCreate(osThread(myTaskInterior), NULL);

  /* definition and creation of myTaskProximity */
  osThreadDef(myTaskProximity, StartTaskProximity, osPriorityAboveNormal, 0, 128);
  myTaskProximityHandle = osThreadCreate(osThread(myTaskProximity), NULL);

  /* definition and creation of myTaskWiper */
  osThreadDef(myTaskWiper, StartTaskWiper, osPriorityNormal, 0, 128);
  myTaskWiperHandle = osThreadCreate(osThread(myTaskWiper), NULL);

  /* definition and creation of myTaskBlinkerL */
  osThreadDef(myTaskBlinkerL, StartTaskBlinkersLeft, osPriorityNormal, 0, 128);
  myTaskBlinkerLHandle = osThreadCreate(osThread(myTaskBlinkerL), NULL);

  /* definition and creation of myTaskLightHigh */
  osThreadDef(myTaskLightHigh, StartTaskLightsHigh, osPriorityNormal, 0, 128);
  myTaskLightHighHandle = osThreadCreate(osThread(myTaskLightHigh), NULL);

  /* definition and creation of myTaskClosedRF */
  osThreadDef(myTaskClosedRF, StartTaskClosedRF, osPriorityNormal, 0, 128);
  myTaskClosedRFHandle = osThreadCreate(osThread(myTaskClosedRF), NULL);

  /* definition and creation of myTaskClosedRR */
  osThreadDef(myTaskClosedRR, StartTaskClosedRR, osPriorityNormal, 0, 128);
  myTaskClosedRRHandle = osThreadCreate(osThread(myTaskClosedRR), NULL);

  /* definition and creation of myTaskClosedLF */
  osThreadDef(myTaskClosedLF, StartTaskClosedLF, osPriorityNormal, 0, 128);
  myTaskClosedLFHandle = osThreadCreate(osThread(myTaskClosedLF), NULL);

  /* definition and creation of myTaskClosedLR */
  osThreadDef(myTaskClosedLR, StartTaskClosedLR, osPriorityNormal, 0, 128);
  myTaskClosedLRHandle = osThreadCreate(osThread(myTaskClosedLR), NULL);

  /* definition and creation of myTaskLockedRF */
  osThreadDef(myTaskLockedRF, StartTaskLockedRF, osPriorityNormal, 0, 128);
  myTaskLockedRFHandle = osThreadCreate(osThread(myTaskLockedRF), NULL);

  /* definition and creation of myTaskLockedRR */
  osThreadDef(myTaskLockedRR, StartTaskLockedRR, osPriorityNormal, 0, 128);
  myTaskLockedRRHandle = osThreadCreate(osThread(myTaskLockedRR), NULL);

  /* definition and creation of myTaskLockedLF */
  osThreadDef(myTaskLockedLF, StartTaskLockedLF, osPriorityNormal, 0, 128);
  myTaskLockedLFHandle = osThreadCreate(osThread(myTaskLockedLF), NULL);

  /* definition and creation of myTaskLockedLR */
  osThreadDef(myTaskLockedLR, StartTaskLockedLR, osPriorityNormal, 0, 128);
  myTaskLockedLRHandle = osThreadCreate(osThread(myTaskLockedLR), NULL);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_UART4
                              |RCC_PERIPHCLK_I2C2;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00303D5B;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* TIM9 init function */
static void MX_TIM9_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 0;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim9);

}

/* TIM11 init function */
static void MX_TIM11_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;

  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 0;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim11) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim11);

}

/* TIM12 init function */
static void MX_TIM12_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 0;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 0;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim12);

}

/* TIM13 init function */
static void MX_TIM13_Init(void)
{

  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 0;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 0;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM14 init function */
static void MX_TIM14_Init(void)
{

  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 0;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* UART4 init function */
static void MX_UART4_Init(void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  huart4.AdvancedInit.AutoBaudRateMode = UART_ADVFEATURE_AUTOBAUDRATE_ON0X55FRAME;
  if (HAL_LIN_Init(&huart4, UART_LINBREAKDETECTLENGTH_10B) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_7B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through 
        * the Code Generation settings)
     PC1   ------> ETH_MDC
     PA1   ------> ETH_REF_CLK
     PA2   ------> ETH_MDIO
     PA7   ------> ETH_CRS_DV
     PC4   ------> ETH_RXD0
     PC5   ------> ETH_RXD1
     PB13   ------> ETH_TXD1
     PA8   ------> USB_OTG_FS_SOF
     PA9   ------> USB_OTG_FS_VBUS
     PA10   ------> USB_OTG_FS_ID
     PA11   ------> USB_OTG_FS_DM
     PA12   ------> USB_OTG_FS_DP
     PG11   ------> ETH_TX_EN
     PG13   ------> ETH_TXD0
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, RESET_N_Pin|LF_BLINKER_Pin|RF_BLINKER_Pin|LM_BLINKER_Pin 
                          |WIPER_0_Pin|R_HIGH_BEAM_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, LF_DOOR_CLOSED_Pin|R_LOW_BEAM_Pin|L_LOW_BEAM_Pin|LF_DOOR_LOCKED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RF_DOOR_CLOSED_Pin|R_REAR_LIGHT_Pin|R_STOP_LIGHT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, L_REAR_LIGHT_Pin|WIPER_3_Pin|LD3_Pin|LR_BLINKER_Pin 
                          |LR_DOOR_LOCKED_Pin|M_STOP_LIGHT_Pin|LD2_Pin|RR_DOOR_LOCKED_Pin 
                          |RR_BLINKER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, RM_BLINKER_Pin|RF_DOOR_LOCKED_Pin|L_HIGH_BEAM_Pin|LIN_SLP_N_Pin 
                          |CHANGE_N_Pin|LR_DOOR_CLOSED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, WIPER_2_Pin|L_STOP_LIGHT_Pin|WIPER_1_Pin|RR_DOOR_CLOSED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE4 PE5 PE7 
                           PE8 PE12 PE15 PE0 
                           PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7 
                          |GPIO_PIN_8|GPIO_PIN_12|GPIO_PIN_15|GPIO_PIN_0 
                          |GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : RESET_N_Pin LF_BLINKER_Pin RF_BLINKER_Pin LM_BLINKER_Pin 
                           WIPER_0_Pin R_HIGH_BEAM_Pin */
  GPIO_InitStruct.Pin = RESET_N_Pin|LF_BLINKER_Pin|RF_BLINKER_Pin|LM_BLINKER_Pin 
                          |WIPER_0_Pin|R_HIGH_BEAM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PF2 PF4 PF5 PF6 
                           PF10 PF11 PF12 PF13 
                           PF14 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13 
                          |GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : LF_DOOR_CLOSED_Pin R_LOW_BEAM_Pin L_LOW_BEAM_Pin LF_DOOR_LOCKED_Pin */
  GPIO_InitStruct.Pin = LF_DOOR_CLOSED_Pin|R_LOW_BEAM_Pin|L_LOW_BEAM_Pin|LF_DOOR_LOCKED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC2 PC3 PC10 
                           PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_10 
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RF_DOOR_CLOSED_Pin R_REAR_LIGHT_Pin R_STOP_LIGHT_Pin */
  GPIO_InitStruct.Pin = RF_DOOR_CLOSED_Pin|R_REAR_LIGHT_Pin|R_STOP_LIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB2 PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PG0 PG1 PG2 PG3 
                           PG4 PG5 PG8 PG9 
                           PG10 PG12 PG14 PG15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9 
                          |GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : L_REAR_LIGHT_Pin WIPER_3_Pin LD3_Pin LR_BLINKER_Pin 
                           LR_DOOR_LOCKED_Pin M_STOP_LIGHT_Pin LD2_Pin RR_DOOR_LOCKED_Pin 
                           RR_BLINKER_Pin */
  GPIO_InitStruct.Pin = L_REAR_LIGHT_Pin|WIPER_3_Pin|LD3_Pin|LR_BLINKER_Pin 
                          |LR_DOOR_LOCKED_Pin|M_STOP_LIGHT_Pin|LD2_Pin|RR_DOOR_LOCKED_Pin 
                          |RR_BLINKER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PD10 PD11 PD15 PD0 
                           PD1 PD2 PD3 PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_15|GPIO_PIN_0 
                          |GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : RM_BLINKER_Pin RF_DOOR_LOCKED_Pin L_HIGH_BEAM_Pin LIN_SLP_N_Pin 
                           CHANGE_N_Pin LR_DOOR_CLOSED_Pin */
  GPIO_InitStruct.Pin = RM_BLINKER_Pin|RF_DOOR_LOCKED_Pin|L_HIGH_BEAM_Pin|LIN_SLP_N_Pin 
                          |CHANGE_N_Pin|LR_DOOR_CLOSED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : WIPER_2_Pin L_STOP_LIGHT_Pin WIPER_1_Pin RR_DOOR_CLOSED_Pin */
  GPIO_InitStruct.Pin = WIPER_2_Pin|L_STOP_LIGHT_Pin|WIPER_1_Pin|RR_DOOR_CLOSED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void setPWM(TIM_HandleTypeDef timer, uint32_t channel, uint16_t period,
uint16_t pulse)
{
 HAL_TIM_PWM_Stop(&timer, channel); // stop generation of pwm
 TIM_OC_InitTypeDef sConfigOC;
 timer.Init.Period = period; // set the period duration
 HAL_TIM_PWM_Init(&timer); // reinititialise with new period value
 sConfigOC.OCMode = TIM_OCMODE_PWM1;
 sConfigOC.Pulse = pulse; // set the pulse duration
 sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
 sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
 HAL_TIM_PWM_ConfigChannel(&timer, &sConfigOC, channel);
 HAL_TIM_PWM_Start(&timer, channel); // start pwm generation
}

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
	char *message, *str;
	char ID, subID;
	
	ID = 0x01;
	subID = 0x01;
	dataField[0] = '1';
	dataField[1] = '\0';
	
  /* Infinite loop */
  for(;;)
  {
		xSemaphoreTake(myMutex01Handle, portMAX_DELAY);
		//if(HAL_UART_Receive(&huart3, (uint8_t *) message, 100, 10) == HAL_OK)
		//{
			xSemaphoreGive(myMutex01Handle);
			// str = stringSplit(message);
			xSemaphoreTake(myMutex02Handle, portMAX_DELAY);
			 
			// subID = str[1];
			// dataField = str[2];
			xSemaphoreGive(myMutex02Handle);
			switch(ID)
			{
				case BLINKER_ID:
					if(subID == R_BLINKER_SUB_ID){
						osThreadSetPriority(myTaskBlinkerRHandle, osPriorityAboveNormal);
					} else if(subID == L_BLINKER_SUB_ID){
						osThreadSetPriority(myTaskBlinkerLHandle, osPriorityAboveNormal);
					}
					break;
				case LIGHTS_ID:
					if(subID == LOW_BEAM_SUB_ID){
						osThreadSetPriority(myTaskLightLowHandle, osPriorityAboveNormal);
					} else if(subID == HIGH_BEAM_SUB_ID){
						osThreadSetPriority(myTaskLightHighHandle, osPriorityAboveNormal);
					}
					break;
				case STOP_LIGHT_ID:
					if(subID == STOP_LIGHT_SUB_ID){
						osThreadSetPriority(myTaskStopLightHandle, osPriorityAboveNormal);
					}
					break;
				case INTERIOR_LIGHT_ID:
					if(subID == INTERIOR_LIGHT_SUB_ID){
						osThreadSetPriority(myTaskInteriorHandle, osPriorityAboveNormal);
					}
					break;
				case WIPER_ID:
					if(subID == WIPER_SUB_ID){
						osThreadSetPriority(myTaskWiperHandle, osPriorityAboveNormal);
					}
					break;
				case DOOR_CLOSED_ID:
					switch(subID){
						case RF_DOOR_SUB_ID:
							osThreadSetPriority(myTaskClosedRFHandle, osPriorityAboveNormal);
							break;
						case RR_DOOR_SUB_ID:
							osThreadSetPriority(myTaskClosedRRHandle, osPriorityAboveNormal);
							break;
						case LF_DOOR_SUB_ID:
							osThreadSetPriority(myTaskClosedLFHandle, osPriorityAboveNormal);
							break;
						case LR_DOOR_SUB_ID:
							osThreadSetPriority(myTaskClosedLRHandle, osPriorityAboveNormal);
							break;
					}
					break;
				case DOOR_LOCKED_ID:
					switch(subID){
						case RF_DOOR_SUB_ID:
							osThreadSetPriority(myTaskLockedRFHandle, osPriorityAboveNormal);
							break;
						case RR_DOOR_SUB_ID:
							osThreadSetPriority(myTaskLockedRRHandle, osPriorityAboveNormal);
							break;
						case LF_DOOR_SUB_ID:
							osThreadSetPriority(myTaskLockedLFHandle, osPriorityAboveNormal);
							break;
						case LR_DOOR_SUB_ID:
							osThreadSetPriority(myTaskLockedLRHandle, osPriorityAboveNormal);
							break;
					}
					break;
			}
			ID++;
			if (ID == 0x08)
				ID = 0x01;
			xSemaphoreTake(myMutex02Handle, portMAX_DELAY);
			if(dataField[1] == '1')
				dataField[1] = '0';
			else
				dataField[1] = '1';
			xSemaphoreGive(myMutex02Handle);
		//}
		HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
		xSemaphoreGive(myMutex01Handle);
  }
  /* USER CODE END 5 */ 
}

/* StartTaskLightsLow function */
void StartTaskLightsLow(void const * argument)
{
  /* USER CODE BEGIN StartTaskLightsLow */
	uint8_t *tempStr;
	
	xSemaphoreTake(myMutex02Handle, portMAX_DELAY);
	tempStr = dataField;
	xSemaphoreGive(myMutex02Handle);
	
  /* Infinite loop */
  for(;;)
  {
		if (tempStr[0] == '1') {
			HAL_GPIO_WritePin(R_LOW_BEAM_GPIO_Port, R_LOW_BEAM_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(R_REAR_LIGHT_GPIO_Port, R_REAR_LIGHT_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(L_LOW_BEAM_GPIO_Port, L_LOW_BEAM_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(L_REAR_LIGHT_GPIO_Port, L_REAR_LIGHT_Pin, GPIO_PIN_SET);
		} else if (tempStr[0] == '0') {
			HAL_GPIO_WritePin(R_LOW_BEAM_GPIO_Port, R_LOW_BEAM_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(R_REAR_LIGHT_GPIO_Port, R_REAR_LIGHT_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(L_LOW_BEAM_GPIO_Port, L_LOW_BEAM_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(L_REAR_LIGHT_GPIO_Port, L_REAR_LIGHT_Pin, GPIO_PIN_RESET);		
		} else {
			printf("Sent wrong data byte!\n");
		}
		osThreadSetPriority(myTaskLightLowHandle, osPriorityNormal);
    osDelay(100);
  }
  /* USER CODE END StartTaskLightsLow */
}

/* StartTaskBlinkersRight function */
void StartTaskBlinkersRight(void const * argument)
{
  /* USER CODE BEGIN StartTaskBlinkersRight */
	// we declare it static for not losing data when changing tasks
	static uint8_t *tempRBlinkerStr;
	
	xSemaphoreTake(myMutex02Handle, portMAX_DELAY);
	tempRBlinkerStr = dataField;
	xSemaphoreGive(myMutex02Handle);
  /* Infinite loop */
  for(;;)
  {
		if (tempRBlinkerStr[0] == '1'){
			if (flag == 0) {
				HAL_GPIO_WritePin(RF_BLINKER_GPIO_Port, RF_BLINKER_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(RM_BLINKER_GPIO_Port, RM_BLINKER_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(RR_BLINKER_GPIO_Port, RR_BLINKER_Pin, GPIO_PIN_SET);
				flag = 1;
			} else {
				HAL_GPIO_WritePin(RF_BLINKER_GPIO_Port, RF_BLINKER_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(RM_BLINKER_GPIO_Port, RM_BLINKER_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(RR_BLINKER_GPIO_Port, RR_BLINKER_Pin, GPIO_PIN_RESET);
				flag = 0;
			}
		} else if (tempRBlinkerStr[0] == '0'){
			HAL_GPIO_WritePin(RF_BLINKER_GPIO_Port, RF_BLINKER_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RM_BLINKER_GPIO_Port, RM_BLINKER_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RR_BLINKER_GPIO_Port, RR_BLINKER_Pin, GPIO_PIN_RESET);
			flag = 0;
			osThreadSetPriority(myTaskBlinkerRHandle, osPriorityNormal);
		}
		
    osDelay(500);
  }
  /* USER CODE END StartTaskBlinkersRight */
}

/* StartTaskStopLights function */
void StartTaskStopLights(void const * argument)
{
  /* USER CODE BEGIN StartTaskStopLights */
	uint8_t *tempStr;
	
	xSemaphoreTake(myMutex02Handle, portMAX_DELAY);
	tempStr = dataField;
	xSemaphoreGive(myMutex02Handle);
  /* Infinite loop */
  for(;;)
  {
		if(tempStr[0] == '1'){
			HAL_GPIO_WritePin(R_STOP_LIGHT_GPIO_Port, R_STOP_LIGHT_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(M_STOP_LIGHT_GPIO_Port, M_STOP_LIGHT_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(L_STOP_LIGHT_GPIO_Port, L_STOP_LIGHT_Pin, GPIO_PIN_SET);
		} else if(tempStr[0] == '0') {
			HAL_GPIO_WritePin(R_STOP_LIGHT_GPIO_Port, R_STOP_LIGHT_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(M_STOP_LIGHT_GPIO_Port, M_STOP_LIGHT_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(L_STOP_LIGHT_GPIO_Port, L_STOP_LIGHT_Pin, GPIO_PIN_RESET);
		}
		osThreadSetPriority(myTaskStopLightHandle, osPriorityNormal);
    osDelay(100);
  }
  /* USER CODE END StartTaskStopLights */
}

/* StartTaskInterior function */
void StartTaskInterior(void const * argument)
{
  /* USER CODE BEGIN StartTaskInterior */
	static int R;
	static int G;
	static int B;
	uint8_t onoff;
	static int incDecR = 1;
	static int incDecG = 1;
	static int incDecB = 1;
	uint8_t *tempStr;
	
	xSemaphoreTake(myMutex02Handle, portMAX_DELAY);
	tempStr = dataField;
	xSemaphoreGive(myMutex02Handle);
	
	onoff = tempStr[0];
	R = tempStr[1];
	G = tempStr[2];
	B = tempStr[3];
  /* Infinite loop */
 for(;;)
  {
		if (onoff == '1'){
			setPWM(htim3, TIM_CHANNEL_4, 255, R);
			setPWM(htim12, TIM_CHANNEL_2, 255, G);
			setPWM(htim9, TIM_CHANNEL_2, 255, B);
			
			R+=incDecR;
			G+=incDecG;
			B+=incDecB;
			
			if(R==256){
				incDecR = -1;
			}else if(R==0){
				incDecR = 1;
			}
		
			if(G==256){
				incDecG = -1;
			}else if(G==0){
				incDecG = 1;
			}
		
			if(B==256){
				incDecB = -1;
			}else if(B==0){
				incDecB = 1;
			}
		} else if (onoff == '0'){
			osThreadSetPriority(myTaskInteriorHandle, osPriorityNormal);
		}
    osDelay(10);
 }
    
  
  /* USER CODE END StartTaskInterior */
}

/* StartTaskProximity function */
void StartTaskProximity(void const * argument)
{
  /* USER CODE BEGIN StartTaskProximity */
	//uint8_t testData[1] = {0x05};
	//uint8_t check[1];
	uint8_t tresholds[12];
	
	//HAL_I2C_Mem_Write(&hi2c2, I2C_ADDRESS << 1, 0x06, I2C_MEMADD_SIZE_8BIT, testData, 1, 100);
	//HAL_I2C_Mem_Read(&hi2c2, I2C_ADDRESS << 1, 0x02, I2C_MEMADD_SIZE_8BIT, check, 1, 100 );
	
	//while(check[0] & (KEY_MASK << 7));

	HAL_I2C_Mem_Read(&hi2c2, I2C_ADDRESS << 1, 16, I2C_MEMADD_SIZE_8BIT, tresholds, 12, 100);
  /* Infinite loop */
  for(;;)
  {
		if(HAL_I2C_Mem_Read(&hi2c2, I2C_ADDRESS << 1, 3, I2C_MEMADD_SIZE_8BIT, data, 2, 100) == HAL_OK){
		//HAL_I2C_Mem_Read(&hi2c2, I2C_ADDRESS << 1, (uint16_t) 0x03, I2C_MEMADD_SIZE_8BIT, data, 2, 100);
			if((data[0] & KEY_MASK) != 0)
				HAL_GPIO_WritePin(LR_DOOR_CLOSED_GPIO_Port, LR_DOOR_CLOSED_Pin, GPIO_PIN_SET);
			else 
				HAL_GPIO_WritePin(LR_DOOR_CLOSED_GPIO_Port, LR_DOOR_CLOSED_Pin, GPIO_PIN_RESET);
			if((data[0] & (KEY_MASK << 4)) != 0)
				HAL_GPIO_WritePin(RR_DOOR_CLOSED_GPIO_Port, RR_DOOR_CLOSED_Pin, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(RR_DOOR_CLOSED_GPIO_Port, RR_DOOR_CLOSED_Pin, GPIO_PIN_RESET);
			if((data[0] & (KEY_MASK << 5)) != 0)
				HAL_GPIO_WritePin(LF_DOOR_CLOSED_GPIO_Port, LF_DOOR_CLOSED_Pin, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(LF_DOOR_CLOSED_GPIO_Port, LF_DOOR_CLOSED_Pin, GPIO_PIN_RESET);
			if((data[1] & (KEY_MASK << 1)) != 0)
				HAL_GPIO_WritePin(RR_DOOR_LOCKED_GPIO_Port, RR_DOOR_LOCKED_Pin, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(RR_DOOR_LOCKED_GPIO_Port, RR_DOOR_LOCKED_Pin, GPIO_PIN_RESET);
			if((data[1] & (KEY_MASK << 2)) != 0)
				HAL_GPIO_WritePin(LF_DOOR_LOCKED_GPIO_Port, LF_DOOR_LOCKED_Pin, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(LF_DOOR_LOCKED_GPIO_Port, LF_DOOR_LOCKED_Pin, GPIO_PIN_RESET);
			if((data[1] & (KEY_MASK << 3)) != 0)
				HAL_GPIO_WritePin(LR_DOOR_LOCKED_GPIO_Port, LR_DOOR_LOCKED_Pin, GPIO_PIN_SET);
			else 
				HAL_GPIO_WritePin(LR_DOOR_LOCKED_GPIO_Port, LR_DOOR_LOCKED_Pin, GPIO_PIN_RESET);
		}
		HAL_I2C_Mem_Read(&hi2c2, I2C_ADDRESS << 1, (uint16_t) 52, I2C_MEMADD_SIZE_8BIT, analogData, 24, 100);
		
    osDelay(1000);
  }
  /* USER CODE END StartTaskProximity */
}

/* StartTaskWiper function */
void StartTaskWiper(void const * argument)
{
  /* USER CODE BEGIN StartTaskWiper */
	uint8_t *tempStr, freq, onoff;
	uint16_t period = 1000;
	
	xSemaphoreTake(myMutex02Handle, portMAX_DELAY);
	tempStr = dataField;
	xSemaphoreGive(myMutex02Handle);
	onoff = tempStr[0];
	freq = tempStr[1];
	
	// period is time in which all 4 leds turn on and all 4 turn off
	// period = scalePeriod(freq);
  /* Infinite loop */
  for(;;)
  {
		if (onoff){
			switch (wiperFlag){
				case 0:
					HAL_GPIO_WritePin(WIPER_3_GPIO_Port, WIPER_3_Pin, GPIO_PIN_SET);
					wiperFlag++;
					osDelay(period/8);
					break;
				case 1:
					HAL_GPIO_WritePin(WIPER_2_GPIO_Port, WIPER_2_Pin, GPIO_PIN_SET);
					wiperFlag++;
					osDelay(period/8);
					break;
				case 2:
					HAL_GPIO_WritePin(WIPER_1_GPIO_Port, WIPER_1_Pin, GPIO_PIN_SET);
					wiperFlag++;
					osDelay(period/8);
					break;
				case 3:
					HAL_GPIO_WritePin(WIPER_0_GPIO_Port, WIPER_0_Pin, GPIO_PIN_SET);
					wiperFlag++;
					osDelay(period/8);
					break;
				case 4:
					HAL_GPIO_WritePin(WIPER_0_GPIO_Port, WIPER_0_Pin, GPIO_PIN_RESET);
					wiperFlag++;
					osDelay(period/8);
					break;
				case 5:
					HAL_GPIO_WritePin(WIPER_1_GPIO_Port, WIPER_1_Pin, GPIO_PIN_RESET);
					wiperFlag++;
					osDelay(period/8);
					break;
				case 6:
					HAL_GPIO_WritePin(WIPER_2_GPIO_Port, WIPER_2_Pin, GPIO_PIN_RESET);
					wiperFlag++;
					osDelay(period/8);
					break;
				case 7:
					HAL_GPIO_WritePin(WIPER_3_GPIO_Port, WIPER_3_Pin, GPIO_PIN_RESET);
					wiperFlag++;
					osDelay(period/8);
					break;
				default:
					wiperFlag = 0;
					osDelay(period/2);
			} 
		} else {
			HAL_GPIO_WritePin(WIPER_3_GPIO_Port, WIPER_3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(WIPER_2_GPIO_Port, WIPER_2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(WIPER_1_GPIO_Port, WIPER_1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(WIPER_0_GPIO_Port, WIPER_0_Pin, GPIO_PIN_RESET);
			osDelay(period/2);
		}
    //osDelay(100);
  }
  /* USER CODE END StartTaskWiper */
}

/* StartTaskBlinkersLeft function */
void StartTaskBlinkersLeft(void const * argument)
{
  /* USER CODE BEGIN StartTaskBlinkersLeft */
  static uint8_t *tempLBlinkerStr;
	
	xSemaphoreTake(myMutex02Handle, portMAX_DELAY);
	tempLBlinkerStr = dataField;
	xSemaphoreGive(myMutex02Handle);
  /* Infinite loop */
  for(;;)
  {
		if (tempLBlinkerStr[0] == '1'){
			if (!flag) {
				HAL_GPIO_WritePin(LF_BLINKER_GPIO_Port, LF_BLINKER_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LM_BLINKER_GPIO_Port, LM_BLINKER_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LR_BLINKER_GPIO_Port, LR_BLINKER_Pin, GPIO_PIN_SET);
				flag = 1;
			} else {
				HAL_GPIO_WritePin(LF_BLINKER_GPIO_Port, LF_BLINKER_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LM_BLINKER_GPIO_Port, LM_BLINKER_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LR_BLINKER_GPIO_Port, LR_BLINKER_Pin, GPIO_PIN_RESET);
				flag = 0;
			}
		} else if (tempLBlinkerStr[0] == '0'){
			HAL_GPIO_WritePin(LF_BLINKER_GPIO_Port, LF_BLINKER_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LM_BLINKER_GPIO_Port, LM_BLINKER_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LR_BLINKER_GPIO_Port, LR_BLINKER_Pin, GPIO_PIN_RESET);
			flag = 0;
			osThreadSetPriority(myTaskBlinkerLHandle, osPriorityNormal);
		}
		
		osDelay(500);
	}
  /* USER CODE END StartTaskBlinkersLeft */
}

/* StartTaskLightsHigh function */
void StartTaskLightsHigh(void const * argument)
{
  /* USER CODE BEGIN StartTaskLightsHigh */
  uint8_t *tempStr;
	
	xSemaphoreTake(myMutex02Handle, portMAX_DELAY);
	tempStr = dataField;
	xSemaphoreGive(myMutex02Handle);
	
  /* Infinite loop */
  for(;;)
  {
		if (tempStr[0] == '1') {
			HAL_GPIO_WritePin(R_HIGH_BEAM_GPIO_Port, R_HIGH_BEAM_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(R_REAR_LIGHT_GPIO_Port, R_REAR_LIGHT_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(L_HIGH_BEAM_GPIO_Port, L_HIGH_BEAM_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(L_REAR_LIGHT_GPIO_Port, L_REAR_LIGHT_Pin, GPIO_PIN_SET);
		} else if (tempStr[0] == '0') {
			HAL_GPIO_WritePin(R_HIGH_BEAM_GPIO_Port, R_HIGH_BEAM_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(R_REAR_LIGHT_GPIO_Port, R_REAR_LIGHT_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(L_HIGH_BEAM_GPIO_Port, L_HIGH_BEAM_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(L_REAR_LIGHT_GPIO_Port, L_REAR_LIGHT_Pin, GPIO_PIN_RESET);		
		} else {
			printf("Sent wrong data byte!\n");
		}
		osThreadSetPriority(myTaskLightHighHandle, osPriorityNormal);
    osDelay(100);
  }
  /* USER CODE END StartTaskLightsHigh */
}

/* StartTaskClosedRF function */
void StartTaskClosedRF(void const * argument)
{
  /* USER CODE BEGIN StartTaskClosedRF */
	uint8_t *tempStr;
	
	xSemaphoreTake(myMutex02Handle, portMAX_DELAY);
	tempStr = dataField;
	xSemaphoreGive(myMutex02Handle);
  /* Infinite loop */
  for(;;)
  {
		if(tempStr[0] == '1'){
			HAL_GPIO_WritePin(RF_DOOR_CLOSED_GPIO_Port, RF_DOOR_CLOSED_Pin, GPIO_PIN_SET);
		} else if (tempStr[0] == '0'){
			HAL_GPIO_WritePin(RF_DOOR_CLOSED_GPIO_Port, RF_DOOR_CLOSED_Pin, GPIO_PIN_RESET);
		}
		osThreadSetPriority(myTaskClosedRFHandle, osPriorityNormal);
    osDelay(100);
  }
  /* USER CODE END StartTaskClosedRF */
}

/* StartTaskClosedRR function */
void StartTaskClosedRR(void const * argument)
{
  /* USER CODE BEGIN StartTaskClosedRR */
  uint8_t *tempStr;
	
	xSemaphoreTake(myMutex02Handle, portMAX_DELAY);
	tempStr = dataField;
	xSemaphoreGive(myMutex02Handle);
  /* Infinite loop */
  for(;;)
  {
		if(tempStr[0] == '1'){
			HAL_GPIO_WritePin(RR_DOOR_CLOSED_GPIO_Port, RR_DOOR_CLOSED_Pin, GPIO_PIN_SET);
		} else if (tempStr[0] == '0'){
			HAL_GPIO_WritePin(RR_DOOR_CLOSED_GPIO_Port, RR_DOOR_CLOSED_Pin, GPIO_PIN_RESET);
		}
		osThreadSetPriority(myTaskClosedRRHandle, osPriorityNormal);
    osDelay(100);
  }
  /* USER CODE END StartTaskClosedRR */
}

/* StartTaskClosedLF function */
void StartTaskClosedLF(void const * argument)
{
  /* USER CODE BEGIN StartTaskClosedLF */
  uint8_t *tempStr;
	
	xSemaphoreTake(myMutex02Handle, portMAX_DELAY);
	tempStr = dataField;
	xSemaphoreGive(myMutex02Handle);
  /* Infinite loop */
  for(;;)
  {
		if(tempStr[0] == '1'){
			HAL_GPIO_WritePin(LF_DOOR_CLOSED_GPIO_Port, LF_DOOR_CLOSED_Pin, GPIO_PIN_SET);
		} else if (tempStr[0] == '0'){
			HAL_GPIO_WritePin(LF_DOOR_CLOSED_GPIO_Port, LF_DOOR_CLOSED_Pin, GPIO_PIN_RESET);
		}
		osThreadSetPriority(myTaskClosedLFHandle, osPriorityNormal);
    osDelay(100);
  }
  /* USER CODE END StartTaskClosedLF */
}

/* StartTaskClosedLR function */
void StartTaskClosedLR(void const * argument)
{
  /* USER CODE BEGIN StartTaskClosedLR */
  uint8_t *tempStr;
	
	xSemaphoreTake(myMutex02Handle, portMAX_DELAY);
	tempStr = dataField;
	xSemaphoreGive(myMutex02Handle);
  /* Infinite loop */
  for(;;)
  {
		if(tempStr[0] == '1'){
			HAL_GPIO_WritePin(LR_DOOR_CLOSED_GPIO_Port, LR_DOOR_CLOSED_Pin, GPIO_PIN_SET);
		} else if (tempStr[0] == '0'){
			HAL_GPIO_WritePin(LR_DOOR_CLOSED_GPIO_Port, LR_DOOR_CLOSED_Pin, GPIO_PIN_RESET);
		}
		osThreadSetPriority(myTaskClosedLRHandle, osPriorityNormal);
    osDelay(100);
  }
  /* USER CODE END StartTaskClosedLR */
}

/* StartTaskLockedRF function */
void StartTaskLockedRF(void const * argument)
{
  /* USER CODE BEGIN StartTaskLockedRF */
  uint8_t *tempStr;
	
	xSemaphoreTake(myMutex02Handle, portMAX_DELAY);
	tempStr = dataField;
	xSemaphoreGive(myMutex02Handle);
  /* Infinite loop */
  for(;;)
  {
		if(tempStr[0] == '1'){
			HAL_GPIO_WritePin(RF_DOOR_LOCKED_GPIO_Port, RF_DOOR_LOCKED_Pin, GPIO_PIN_SET);
		} else if (tempStr[0] == '0'){
			HAL_GPIO_WritePin(RF_DOOR_LOCKED_GPIO_Port, RF_DOOR_LOCKED_Pin, GPIO_PIN_RESET);
		}
		osThreadSetPriority(myTaskLockedRFHandle, osPriorityNormal);
    osDelay(100);
  }
  /* USER CODE END StartTaskLockedRF */
}

/* StartTaskLockedRR function */
void StartTaskLockedRR(void const * argument)
{
  /* USER CODE BEGIN StartTaskLockedRR */
  uint8_t *tempStr;
	
	xSemaphoreTake(myMutex02Handle, portMAX_DELAY);
	tempStr = dataField;
	xSemaphoreGive(myMutex02Handle);
  /* Infinite loop */
  for(;;)
  {
		if(tempStr[0] == '1'){
			HAL_GPIO_WritePin(RR_DOOR_LOCKED_GPIO_Port, RR_DOOR_LOCKED_Pin, GPIO_PIN_SET);
		} else if (tempStr[0] == '0'){
			HAL_GPIO_WritePin(RR_DOOR_LOCKED_GPIO_Port, RR_DOOR_LOCKED_Pin, GPIO_PIN_RESET);
		}
		osThreadSetPriority(myTaskLockedRRHandle, osPriorityNormal);
    osDelay(100);
  }
  /* USER CODE END StartTaskLockedRR */
}

/* StartTaskLockedLF function */
void StartTaskLockedLF(void const * argument)
{
  /* USER CODE BEGIN StartTaskLockedLF */
  uint8_t *tempStr;
	
	xSemaphoreTake(myMutex02Handle, portMAX_DELAY);
	tempStr = dataField;
	xSemaphoreGive(myMutex02Handle);
  /* Infinite loop */
  for(;;)
  {
		if(tempStr[0] == '1'){
			HAL_GPIO_WritePin(LF_DOOR_LOCKED_GPIO_Port, LF_DOOR_LOCKED_Pin, GPIO_PIN_SET);
		} else if (tempStr[0] == '0'){
			HAL_GPIO_WritePin(LF_DOOR_LOCKED_GPIO_Port, LF_DOOR_LOCKED_Pin, GPIO_PIN_RESET);
		}
		osThreadSetPriority(myTaskLockedLFHandle, osPriorityNormal);
    osDelay(100);
  }
  /* USER CODE END StartTaskLockedLF */
}

/* StartTaskLockedLR function */
void StartTaskLockedLR(void const * argument)
{
  /* USER CODE BEGIN StartTaskLockedLR */
  uint8_t *tempStr;
	
	xSemaphoreTake(myMutex02Handle, portMAX_DELAY);
	tempStr = dataField;
	xSemaphoreGive(myMutex02Handle);
  /* Infinite loop */
  for(;;)
  {
		if(tempStr[0] == '1'){
			HAL_GPIO_WritePin(LR_DOOR_LOCKED_GPIO_Port, LR_DOOR_LOCKED_Pin, GPIO_PIN_SET);
		} else if (tempStr[0] == '0'){
			HAL_GPIO_WritePin(LR_DOOR_LOCKED_GPIO_Port, LR_DOOR_LOCKED_Pin, GPIO_PIN_RESET);
		}
		osThreadSetPriority(myTaskLockedLRHandle, osPriorityNormal);
    osDelay(100);
  }
  /* USER CODE END StartTaskLockedLR */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
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
