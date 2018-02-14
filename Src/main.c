/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
#define DOOR_ID									0x06
#define RF_DOOR_CLOSED_SUB_ID		0x01
#define RR_DOOR_CLOSED_SUB_ID		0x02
#define LF_DOOR_CLOSED_SUB_ID		0x03
#define LR_DOOR_CLOSED_SUB_ID		0x04
#define RF_DOOR_LOCKED_SUB_ID		0x11
#define RR_DOOR_LOCKED_SUB_ID		0x12
#define LF_DOOR_LOCKED_SUB_ID		0x13
#define LR_DOOR_LOCKED_SUB_ID		0x14

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

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
osThreadId myTaskCanHandle;
osThreadId myTaskLinHandle;
osThreadId myTaskLightLowHandle;
osThreadId myTaskBlinkerRHandle;
osThreadId myTaskStopLightHandle;
osThreadId myTaskTouchKeysHandle;
osThreadId myTaskI2CHandle;
osThreadId myTaskInteriorHandle;
osThreadId myTaskBuzzerHandle;
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
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
void StartTaskCan(void const * argument);
void StartTaskLin(void const * argument);
void StartTaskLightsLow(void const * argument);
void StartTaskBlinkersRight(void const * argument);
void StartTaskStopLights(void const * argument);
void StartTaskTouchKeys(void const * argument);
void StartTaskI2C(void const * argument);
void StartTaskInterior(void const * argument);
void StartTaskBuzzer(void const * argument);
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
  MX_GPIO_Init();
  MX_CAN1_Init();
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

	
		HAL_UART_Receive_IT(&huart4,Test2,1);
	
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

  /* definition and creation of myTaskCan */
  osThreadDef(myTaskCan, StartTaskCan, osPriorityNormal, 0, 128);
  myTaskCanHandle = osThreadCreate(osThread(myTaskCan), NULL);

  /* definition and creation of myTaskLin */
  osThreadDef(myTaskLin, StartTaskLin, osPriorityNormal, 0, 128);
  myTaskLinHandle = osThreadCreate(osThread(myTaskLin), NULL);

  /* definition and creation of myTaskLightLow */
  osThreadDef(myTaskLightLow, StartTaskLightsLow, osPriorityNormal, 0, 128);
  myTaskLightLowHandle = osThreadCreate(osThread(myTaskLightLow), NULL);

  /* definition and creation of myTaskBlinkerR */
  osThreadDef(myTaskBlinkerR, StartTaskBlinkersRight, osPriorityNormal, 0, 128);
  myTaskBlinkerRHandle = osThreadCreate(osThread(myTaskBlinkerR), NULL);

  /* definition and creation of myTaskStopLight */
  osThreadDef(myTaskStopLight, StartTaskStopLights, osPriorityNormal, 0, 128);
  myTaskStopLightHandle = osThreadCreate(osThread(myTaskStopLight), NULL);

  /* definition and creation of myTaskTouchKeys */
  osThreadDef(myTaskTouchKeys, StartTaskTouchKeys, osPriorityNormal, 0, 128);
  myTaskTouchKeysHandle = osThreadCreate(osThread(myTaskTouchKeys), NULL);

  /* definition and creation of myTaskI2C */
  osThreadDef(myTaskI2C, StartTaskI2C, osPriorityNormal, 0, 128);
  myTaskI2CHandle = osThreadCreate(osThread(myTaskI2C), NULL);

  /* definition and creation of myTaskInterior */
  osThreadDef(myTaskInterior, StartTaskInterior, osPriorityNormal, 0, 128);
  myTaskInteriorHandle = osThreadCreate(osThread(myTaskInterior), NULL);

  /* definition and creation of myTaskBuzzer */
  osThreadDef(myTaskBuzzer, StartTaskBuzzer, osPriorityNormal, 0, 128);
  myTaskBuzzerHandle = osThreadCreate(osThread(myTaskBuzzer), NULL);

  /* definition and creation of myTaskProximity */
  osThreadDef(myTaskProximity, StartTaskProximity, osPriorityNormal, 0, 128);
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

/** System Clock Configuration
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

/* CAN1 init function */
static void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SJW = CAN_SJW_1TQ;
  hcan1.Init.BS1 = CAN_BS1_1TQ;
  hcan1.Init.BS2 = CAN_BS2_1TQ;
  hcan1.Init.TTCM = DISABLE;
  hcan1.Init.ABOM = DISABLE;
  hcan1.Init.AWUM = DISABLE;
  hcan1.Init.NART = DISABLE;
  hcan1.Init.RFLM = DISABLE;
  hcan1.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

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

  TIM_OC_InitTypeDef sConfigOC;

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

  if (HAL_TIM_PWM_Init(&htim13) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim13);

}

/* TIM14 init function */
static void MX_TIM14_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;

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

  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim14);

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
  HAL_GPIO_WritePin(GPIOF, LF_DOOR_CLOSED_Pin|LF_DOOR_LOCKED_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : LF_DOOR_CLOSED_Pin LF_DOOR_LOCKED_Pin */
  GPIO_InitStruct.Pin = LF_DOOR_CLOSED_Pin|LF_DOOR_LOCKED_Pin;
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

  /*Configure GPIO pins : PD10 PD11 PD15 PD2 
                           PD3 PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_15|GPIO_PIN_2 
                          |GPIO_PIN_3|GPIO_PIN_4;
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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	
	if (Test2[0]=='1')
	{
		 HAL_GPIO_TogglePin(LD3_GPIO_Port,LD3_Pin);
	}
 Test2[0] = ' ';
	
//while(	huart4.gState !=  HAL_UART_STATE_READY)
//	{}
	HAL_UART_Receive_IT(&huart4,Test2,1);
	
} 

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static unsigned long last_interrupt_time;
	unsigned long interrupt_time=HAL_GetTick();
	if ((interrupt_time-last_interrupt_time)>250)
	{
	if (GPIO_Pin==USER_Btn_Pin)
	{
		button++;
			if (button==5)
		{
			button=0;
		}
	
	}
}
	last_interrupt_time=HAL_GetTick();
}

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
	char *message, *str;
	
  /* Infinite loop */
  for(;;)
  {
		xSemaphoreTake(myMutex01Handle, portMAX_DELAY);
		if(HAL_UART_Receive(&huart3, (uint8_t *) message, 100, 10) == HAL_OK)
		{
			// str = stringSplit(message);
			xSemaphoreTake(myMutex02Handle, portMAX_DELAY);
			// dataField = str[2];
			xSemaphoreGive(myMutex02Handle);
			switch(str[0])
			{
				case 0x01:
					if(str[1] == 0x01){
						osThreadSetPriority(myTaskBlinkerRHandle, osPriorityAboveNormal);
					} else {
						osThreadSetPriority(myTaskBlinkerLHandle, osPriorityAboveNormal);
					}
					break;
				case 0x02:
					if(str[1] == 0x01){
						osThreadSetPriority(myTaskLightLowHandle, osPriorityAboveNormal);
					} else {
						osThreadSetPriority(myTaskLightHighHandle, osPriorityAboveNormal);
					}
					break;
				case 0x03:
					if(str[1] == 0x01){
						osThreadSetPriority(myTaskStopLightHandle, osPriorityAboveNormal);
					}
					break;
				case 0x04:
					if(str[1] == 0x01){
						osThreadSetPriority(myTaskInteriorHandle, osPriorityAboveNormal);
					}
					break;
				case 0x05:
					if(str[1] == 0x01){
						osThreadSetPriority(myTaskWiperHandle, osPriorityAboveNormal);
					}
					break;
				case 0x06:
					switch(str[1]){
						case 0x01:
							osThreadSetPriority(myTaskClosedRFHandle, osPriorityAboveNormal);
							break;
						case 0x02:
							osThreadSetPriority(myTaskClosedRRHandle, osPriorityAboveNormal);
							break;
						case 0x03:
							osThreadSetPriority(myTaskClosedLFHandle, osPriorityAboveNormal);
							break;
						case 0x04:
							osThreadSetPriority(myTaskClosedLRHandle, osPriorityAboveNormal);
							break;
					}
					break;
				case 0x07:
					switch(str[1]){
						case 0x01:
							osThreadSetPriority(myTaskLockedRFHandle, osPriorityAboveNormal);
							break;
						case 0x02:
							osThreadSetPriority(myTaskLockedRRHandle, osPriorityAboveNormal);
							break;
						case 0x03:
							osThreadSetPriority(myTaskLockedLFHandle, osPriorityAboveNormal);
							break;
						case 0x04:
							osThreadSetPriority(myTaskLockedLRHandle, osPriorityAboveNormal);
							break;
					}
					break;
			}
		}
		HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
		xSemaphoreGive(myMutex01Handle);
    osDelay(100);
  }
  /* USER CODE END 5 */ 
}

/* StartTaskCan function */
void StartTaskCan(void const * argument)
{
  /* USER CODE BEGIN StartTaskCan */
  /* Infinite loop */
  for(;;)
  {
		xSemaphoreTake(myMutex01Handle, portMAX_DELAY);		
		HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
		xSemaphoreGive(myMutex01Handle);
    osDelay(1000);
		
		
  }
  /* USER CODE END StartTaskCan */
}

/* StartTaskLin function */
void StartTaskLin(void const * argument)
{
  /* USER CODE BEGIN StartTaskLin */
  /* Infinite loop */
  for(;;)
  {
		char *message;
		message = createPackage(0x25, 0x21, 0x23, " woow, yay");		
		HAL_UART_Transmit(&huart3,(uint8_t*)message,stringLength(*message),10);
		
		
//		uint8_t Test[5] = "11111"; //Data to send 
//		HAL_UART_Transmit_IT(&huart4,Test,5);// Sending in normal mode
//		uint8_t Test3[5] = "11111";
//		HAL_UART_Transmit(&huart3,Test3,5,10);// Sending in normal mode
		
		osDelay(100);
		
  }
  /* USER CODE END StartTaskLin */
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
    osDelay(1);
  }
  /* USER CODE END StartTaskLightsLow */
}

/* StartTaskBlinkersRight function */
void StartTaskBlinkersRight(void const * argument)
{
  /* USER CODE BEGIN StartTaskBlinkersRight */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTaskBlinkersRight */
}

/* StartTaskStopLights function */
void StartTaskStopLights(void const * argument)
{
  /* USER CODE BEGIN StartTaskStopLights */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTaskStopLights */
}

/* StartTaskTouchKeys function */
void StartTaskTouchKeys(void const * argument)
{
  /* USER CODE BEGIN StartTaskTouchKeys */
  /* Infinite loop */
  for(;;)
  {
    osDelay(100);
  }
  /* USER CODE END StartTaskTouchKeys */
}

/* StartTaskI2C function */
void StartTaskI2C(void const * argument)
{
  /* USER CODE BEGIN StartTaskI2C */
  /* Infinite loop */
  for(;;)
  {
    osDelay(100);
  }
  /* USER CODE END StartTaskI2C */
}

/* StartTaskInterior function */
void StartTaskInterior(void const * argument)
{
  /* USER CODE BEGIN StartTaskInterior */
	static int R=0;
	static int G=85;
	static int B=170;
	static int incDecR = 1;
	static int incDecG = 1;
	static int incDecB = 1;
  /* Infinite loop */
 for(;;)
  {
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
    osDelay(10);
 }
    
  
  /* USER CODE END StartTaskInterior */
}

/* StartTaskBuzzer function */
void StartTaskBuzzer(void const * argument)
{
  /* USER CODE BEGIN StartTaskBuzzer */
  /* Infinite loop */
  for(;;)
  {
		
			if (button==0)
			{
				setPWM(htim11, TIM_CHANNEL_1, 255, 0);
			}
			if (button==4)
			{
				setPWM(htim11, TIM_CHANNEL_1, 255, 255);
			}
    osDelay(100);
  }
  /* USER CODE END StartTaskBuzzer */
}

/* StartTaskProximity function */
void StartTaskProximity(void const * argument)
{
  /* USER CODE BEGIN StartTaskProximity */
  /* Infinite loop */
  for(;;)
  {
    osDelay(100);
  }
  /* USER CODE END StartTaskProximity */
}

/* StartTaskWiper function */
void StartTaskWiper(void const * argument)
{
  /* USER CODE BEGIN StartTaskWiper */
	int16_t i;
  /* Infinite loop */
  for(;;)
  {   
    if (i<=2)
		{

			if (HAL_GPIO_ReadPin(WIPER_1_GPIO_Port,WIPER_1_Pin)==GPIO_PIN_SET)
			{
				HAL_GPIO_WritePin(WIPER_0_GPIO_Port,WIPER_0_Pin,1);
				i++;
			}
			if (HAL_GPIO_ReadPin(WIPER_2_GPIO_Port,WIPER_2_Pin)==GPIO_PIN_SET)
			{
				HAL_GPIO_WritePin(WIPER_1_GPIO_Port,WIPER_1_Pin, GPIO_PIN_SET);
			}
			if (HAL_GPIO_ReadPin(WIPER_3_GPIO_Port,WIPER_3_Pin)==GPIO_PIN_SET)
			{
				HAL_GPIO_WritePin(WIPER_2_GPIO_Port,WIPER_2_Pin,1);
			}
			if (HAL_GPIO_ReadPin(WIPER_0_GPIO_Port,WIPER_0_Pin)==GPIO_PIN_RESET&&HAL_GPIO_ReadPin(WIPER_2_GPIO_Port,WIPER_2_Pin)==GPIO_PIN_RESET&&HAL_GPIO_ReadPin(WIPER_1_GPIO_Port,WIPER_1_Pin)==GPIO_PIN_RESET)
			{
				HAL_GPIO_WritePin(WIPER_3_GPIO_Port,WIPER_3_Pin,1);
			}
			
		}
		if (i==3)
		{
			  if (HAL_GPIO_ReadPin(WIPER_2_GPIO_Port,WIPER_2_Pin)==GPIO_PIN_RESET)
				
			{
				HAL_GPIO_WritePin(WIPER_3_GPIO_Port,WIPER_3_Pin,0);
					i=0;
			}
			if (HAL_GPIO_ReadPin(WIPER_1_GPIO_Port,WIPER_1_Pin)==GPIO_PIN_RESET)
			{
				HAL_GPIO_WritePin(WIPER_2_GPIO_Port,WIPER_2_Pin,0);
			}
			if (HAL_GPIO_ReadPin(WIPER_0_GPIO_Port,WIPER_0_Pin)==GPIO_PIN_RESET)
			{
				HAL_GPIO_WritePin(WIPER_1_GPIO_Port,WIPER_1_Pin,0);
			}
			if (HAL_GPIO_ReadPin(WIPER_3_GPIO_Port,WIPER_3_Pin)==GPIO_PIN_SET&&HAL_GPIO_ReadPin(WIPER_2_GPIO_Port,WIPER_2_Pin)==GPIO_PIN_SET&&HAL_GPIO_ReadPin(WIPER_1_GPIO_Port,WIPER_1_Pin)==GPIO_PIN_SET)
			{
				HAL_GPIO_WritePin(WIPER_0_GPIO_Port,WIPER_0_Pin,0);
			}
		}
		
    osDelay(100);
  }
  /* USER CODE END StartTaskWiper */
}

/* StartTaskBlinkersLeft function */
void StartTaskBlinkersLeft(void const * argument)
{
  /* USER CODE BEGIN StartTaskBlinkersLeft */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTaskBlinkersLeft */
}

/* StartTaskLightsHigh function */
void StartTaskLightsHigh(void const * argument)
{
  /* USER CODE BEGIN StartTaskLightsHigh */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTaskLightsHigh */
}

/* StartTaskClosedRF function */
void StartTaskClosedRF(void const * argument)
{
  /* USER CODE BEGIN StartTaskClosedRF */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTaskClosedRF */
}

/* StartTaskClosedRR function */
void StartTaskClosedRR(void const * argument)
{
  /* USER CODE BEGIN StartTaskClosedRR */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTaskClosedRR */
}

/* StartTaskClosedLF function */
void StartTaskClosedLF(void const * argument)
{
  /* USER CODE BEGIN StartTaskClosedLF */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTaskClosedLF */
}

/* StartTaskClosedLR function */
void StartTaskClosedLR(void const * argument)
{
  /* USER CODE BEGIN StartTaskClosedLR */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTaskClosedLR */
}

/* StartTaskLockedRF function */
void StartTaskLockedRF(void const * argument)
{
  /* USER CODE BEGIN StartTaskLockedRF */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTaskLockedRF */
}

/* StartTaskLockedRR function */
void StartTaskLockedRR(void const * argument)
{
  /* USER CODE BEGIN StartTaskLockedRR */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTaskLockedRR */
}

/* StartTaskLockedLF function */
void StartTaskLockedLF(void const * argument)
{
  /* USER CODE BEGIN StartTaskLockedLF */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTaskLockedLF */
}

/* StartTaskLockedLR function */
void StartTaskLockedLR(void const * argument)
{
  /* USER CODE BEGIN StartTaskLockedLR */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
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
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
