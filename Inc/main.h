/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define RESET_N_Pin	 										GPIO_PIN_3
#define RESET_N_GPIO_Port 							GPIOE
#define INTERIOR_BLUE_Pin 							GPIO_PIN_6	
#define INTERIOR_BLUE_GPIO_Port 				GPIOE
#define USER_Btn_Pin 										GPIO_PIN_13
#define USER_Btn_GPIO_Port 							GPIOC
#define USER_Btn_EXTI_IRQn 							EXTI15_10_IRQn
#define LF_DOOR_CLOSED_Pin 							GPIO_PIN_3	
#define LF_DOOR_CLOSED_GPIO_Port 				GPIOF
#define BUZZER_Pin 											GPIO_PIN_7
#define BUZZER_GPIO_Port 								GPIOF
#define R_LOW_BEAM_Pin 									GPIO_PIN_8
#define R_LOW_BEAM_GPIO_Port 						GPIOF
#define L_LOW_BEAM_Pin 									GPIO_PIN_9
#define L_LOW_BEAM_GPIO_Port 						GPIOF
#define MCO_Pin 												GPIO_PIN_0
#define MCO_GPIO_Port 									GPIOH
#define RMII_MDC_Pin 										GPIO_PIN_1
#define RMII_MDC_GPIO_Port 							GPIOC
#define RMII_REF_CLK_Pin 								GPIO_PIN_1
#define RMII_REF_CLK_GPIO_Port 					GPIOA
#define RMII_MDIO_Pin 									GPIO_PIN_2
#define RMII_MDIO_GPIO_Port 						GPIOA
#define RF_DOOR_CLOSED_Pin 							GPIO_PIN_3
#define RF_DOOR_CLOSED_GPIO_Port 				GPIOA
#define R_REAR_LIGHT_Pin 								GPIO_PIN_5
#define R_REAR_LIGHT_GPIO_Port 					GPIOA
#define R_STOP_LIGHT_Pin 								GPIO_PIN_6
#define R_STOP_LIGHT_GPIO_Port 					GPIOA
#define RMII_CRS_DV_Pin 								GPIO_PIN_7
#define RMII_CRS_DV_GPIO_Port 					GPIOA
#define RMII_RXD0_Pin 									GPIO_PIN_4
#define RMII_RXD0_GPIO_Port 						GPIOC
#define RMII_RXD1_Pin 									GPIO_PIN_5
#define RMII_RXD1_GPIO_Port 						GPIOC
#define INTERIOR_RED_Pin 								GPIO_PIN_1
#define INTERIOR_RED_GPIO_Port 					GPIOB
#define LF_DOOR_LOCKED_Pin 							GPIO_PIN_15
#define LF_DOOR_LOCKED_GPIO_Port 				GPIOF
#define LF_BLINKER_Pin 									GPIO_PIN_9
#define LF_BLINKER_GPIO_Port 						GPIOE
#define RF_BLINKER_Pin 									GPIO_PIN_10
#define RF_BLINKER_GPIO_Port 						GPIOE
#define LM_BLINKER_Pin 									GPIO_PIN_11
#define LM_BLINKER_GPIO_Port 						GPIOE
#define WIPER_0_Pin 										GPIO_PIN_13
#define WIPER_0_GPIO_Port 							GPIOE
#define R_HIGH_BEAM_Pin 								GPIO_PIN_14
#define R_HIGH_BEAM_GPIO_Port 					GPIOE
#define L_REAR_LIGHT_Pin 								GPIO_PIN_10
#define L_REAR_LIGHT_GPIO_Port 					GPIOB
#define WIPER_3_Pin 										GPIO_PIN_11
#define WIPER_3_GPIO_Port 							GPIOB
#define RMII_TXD1_Pin 									GPIO_PIN_13
#define RMII_TXD1_GPIO_Port 						GPIOB
#define LD3_Pin 												GPIO_PIN_14
#define LD3_GPIO_Port 									GPIOB
#define INTERIOR_GREEN_Pin 							GPIO_PIN_15
#define INTERIOR_GREEN_GPIO_Port 				GPIOB
#define LIN_TX_Pin 											GPIO_PIN_8
#define LIN_TX_GPIO_Port 								GPIOD
#define LIN_RX_Pin 											GPIO_PIN_9
#define LIN_RX_GPIO_Port 								GPIOD
#define RM_BLINKER_Pin 									GPIO_PIN_12
#define RM_BLINKER_GPIO_Port 						GPIOD
#define RF_DOOR_LOCKED_Pin 							GPIO_PIN_13
#define RF_DOOR_LOCKED_GPIO_Port 				GPIOD
#define L_HIGH_BEAM_Pin 								GPIO_PIN_14
#define L_HIGH_BEAM_GPIO_Port 					GPIOD
#define USB_PowerSwitchOn_Pin 					GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port 		GPIOG
#define USB_OverCurrent_Pin 						GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port 			GPIOG
#define WIPER_2_Pin 										GPIO_PIN_6
#define WIPER_2_GPIO_Port 							GPIOC
#define L_STOP_LIGHT_Pin 								GPIO_PIN_7
#define L_STOP_LIGHT_GPIO_Port 					GPIOC
#define WIPER_1_Pin 										GPIO_PIN_8
#define WIPER_1_GPIO_Port 							GPIOC
#define RR_DOOR_CLOSED_Pin 							GPIO_PIN_9
#define RR_DOOR_CLOSED_GPIO_Port 				GPIOC
#define USB_SOF_Pin 										GPIO_PIN_8
#define USB_SOF_GPIO_Port 							GPIOA
#define USB_VBUS_Pin 										GPIO_PIN_9
#define USB_VBUS_GPIO_Port 							GPIOA
#define USB_ID_Pin 											GPIO_PIN_10
#define USB_ID_GPIO_Port 								GPIOA
#define USB_DM_Pin 											GPIO_PIN_11
#define USB_DM_GPIO_Port 								GPIOA
#define USB_DP_Pin 											GPIO_PIN_12
#define USB_DP_GPIO_Port 								GPIOA
#define TMS_Pin 												GPIO_PIN_13
#define TMS_GPIO_Port 									GPIOA
#define TCK_Pin 												GPIO_PIN_14
#define TCK_GPIO_Port 									GPIOA
#define LIN_SLP_N_Pin 									GPIO_PIN_5
#define LIN_SLP_N_GPIO_Port 						GPIOD
#define CHANGE_N_Pin 										GPIO_PIN_6
#define CHANGE_N_GPIO_Port 							GPIOD
#define LR_DOOR_CLOSED_Pin 							GPIO_PIN_7
#define LR_DOOR_CLOSED_GPIO_Port 				GPIOD
#define RMII_TX_EN_Pin 									GPIO_PIN_11
#define RMII_TX_EN_GPIO_Port 						GPIOG
#define RMII_TXD0_Pin 									GPIO_PIN_13
#define RMII_TXD0_GPIO_Port 						GPIOG
#define SW0_Pin 												GPIO_PIN_3
#define SW0_GPIO_Port 									GPIOB
#define LR_BLINKER_Pin 									GPIO_PIN_4
#define LR_BLINKER_GPIO_Port 						GPIOB
#define LR_DOOR_LOCKED_Pin 							GPIO_PIN_5
#define LR_DOOR_LOCKED_GPIO_Port 				GPIOB
#define M_STOP_LIGHT_Pin 								GPIO_PIN_6
#define M_STOP_LIGHT_GPIO_Port 					GPIOB
#define LD2_Pin 												GPIO_PIN_7
#define LD2_GPIO_Port 									GPIOB
#define RR_DOOR_LOCKED_Pin 							GPIO_PIN_8
#define RR_DOOR_LOCKED_GPIO_Port 				GPIOB
#define RR_BLINKER_Pin 									GPIO_PIN_9
#define RR_BLINKER_GPIO_Port 						GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
