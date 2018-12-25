/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

#define RAY_L1_Pin GPIO_PIN_0
#define RAY_L1_GPIO_Port GPIOC
#define RAY_L1_EXTI_IRQn EXTI0_IRQn
#define RAY_L2_Pin GPIO_PIN_1
#define RAY_L2_GPIO_Port GPIOC
#define RAY_L2_EXTI_IRQn EXTI1_IRQn
#define RAY_R1_Pin GPIO_PIN_2
#define RAY_R1_GPIO_Port GPIOC
#define RAY_R1_EXTI_IRQn EXTI2_IRQn
#define RAY_R2_Pin GPIO_PIN_3
#define RAY_R2_GPIO_Port GPIOC
#define RAY_R2_EXTI_IRQn EXTI3_IRQn
#define ENCODER_R2A_Pin GPIO_PIN_0
#define ENCODER_R2A_GPIO_Port GPIOA
#define ENCODER_R2B_Pin GPIO_PIN_1
#define ENCODER_R2B_GPIO_Port GPIOA
#define WIFI_TX_Pin GPIO_PIN_2
#define WIFI_TX_GPIO_Port GPIOA
#define WIFI_RX_Pin GPIO_PIN_3
#define WIFI_RX_GPIO_Port GPIOA
#define ENCODER_R1A_Pin GPIO_PIN_6
#define ENCODER_R1A_GPIO_Port GPIOA
#define ENCODER_R1B_Pin GPIO_PIN_7
#define ENCODER_R1B_GPIO_Port GPIOA
#define PWM_R2F_Pin GPIO_PIN_0
#define PWM_R2F_GPIO_Port GPIOB
#define PWM_R2B_Pin GPIO_PIN_1
#define PWM_R2B_GPIO_Port GPIOB
#define GYRO_TX_Pin GPIO_PIN_10
#define GYRO_TX_GPIO_Port GPIOB
#define GYRO_RX_Pin GPIO_PIN_11
#define GYRO_RX_GPIO_Port GPIOB
#define PWM_L2F_Pin GPIO_PIN_6
#define PWM_L2F_GPIO_Port GPIOC
#define PWM_L2B_Pin GPIO_PIN_7
#define PWM_L2B_GPIO_Port GPIOC
#define PWM_R1F_Pin GPIO_PIN_8
#define PWM_R1F_GPIO_Port GPIOC
#define PWM_R1B_Pin GPIO_PIN_9
#define PWM_R1B_GPIO_Port GPIOC
#define PWM_L1F_Pin GPIO_PIN_8
#define PWM_L1F_GPIO_Port GPIOA
#define BT_TX_Pin GPIO_PIN_9
#define BT_TX_GPIO_Port GPIOA
#define BT_RX_Pin GPIO_PIN_10
#define BT_RX_GPIO_Port GPIOA
#define PWM_L1B_Pin GPIO_PIN_11
#define PWM_L1B_GPIO_Port GPIOA
#define ENCODER_L1A_Pin GPIO_PIN_15
#define ENCODER_L1A_GPIO_Port GPIOA
#define ENCODER_L1B_Pin GPIO_PIN_3
#define ENCODER_L1B_GPIO_Port GPIOB
#define RAY_L3_Pin GPIO_PIN_4
#define RAY_L3_GPIO_Port GPIOB
#define RAY_L3_EXTI_IRQn EXTI4_IRQn
#define RAY_R3_Pin GPIO_PIN_5
#define RAY_R3_GPIO_Port GPIOB
#define RAY_R3_EXTI_IRQn EXTI9_5_IRQn
#define ENCODER_L2A_Pin GPIO_PIN_6
#define ENCODER_L2A_GPIO_Port GPIOB
#define ENCODER_L2B_Pin GPIO_PIN_7
#define ENCODER_L2B_GPIO_Port GPIOB
#define CAR_CASE_GPIO_Pin GPIO_PIN_9
#define CAR_CASE_GPIO_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

//basic 
#define PI 3.14159

#define UART1_BUF_MAX 1024
#define UART2_BUF_MAX 1024
#define UART3_BUF_MAX 1024

//DEBUG 
#define INIT_DEBUG

//	WIFI 

//#define UART_DEBUG 
#define WIFI_DEBUG
//#define WIFI_DECODE_DEBUG

//	control 
//#define CONTROL_DEBUG 
//#define BT_DEBUG 
//#define SPEED_DEBUG 
//#define PID_DEBUG 
//#define PID_DEBUG_L1
//#define PID_DEBUG_R1
//#define PID_DEBUG_L2
//#define PID_DEBUG_R2
//#define RAY_DEBUG
//#define DIRECTION_DEBUG
#define TO_TARGET_DEBUG
			
//	state 
#define STATE_DEBUG
#define ROUTE_DEBUG
#define SORT_DEBUG 



//used in control.c
//#define INIT_ANGLE 0;	//when tuning


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
