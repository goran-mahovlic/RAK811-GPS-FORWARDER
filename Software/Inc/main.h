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
#include "stm32l1xx_hal.h"
#include <stdint.h>
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define RADIO_DIO_4_Pin GPIO_PIN_13
#define RADIO_DIO_4_GPIO_Port GPIOC
#define RADIO_DIO_3_Pin GPIO_PIN_0
#define RADIO_DIO_3_GPIO_Port GPIOH
#define RADIO_XTAL_EN_Pin GPIO_PIN_1
#define RADIO_XTAL_EN_GPIO_Port GPIOH
#define GPS_PPS_PIN_Pin GPIO_PIN_0
#define GPS_PPS_PIN_GPIO_Port GPIOA
#define BAT_LEVEL_PIN_Pin GPIO_PIN_2
#define BAT_LEVEL_PIN_GPIO_Port GPIOA
#define RADIO_DIO_2_Pin GPIO_PIN_3
#define RADIO_DIO_2_GPIO_Port GPIOA
#define RADIO_RF_CTX_PA_Pin GPIO_PIN_4
#define RADIO_RF_CTX_PA_GPIO_Port GPIOA
#define RADIO_SCLK_Pin GPIO_PIN_5
#define RADIO_SCLK_GPIO_Port GPIOA
#define RADIO_MISO_Pin GPIO_PIN_6
#define RADIO_MISO_GPIO_Port GPIOA
#define RADIO_MOSI_Pin GPIO_PIN_7
#define RADIO_MOSI_GPIO_Port GPIOA
#define RADIO_NSS_Pin GPIO_PIN_0
#define RADIO_NSS_GPIO_Port GPIOB
#define RADIO_DIO_1_Pin GPIO_PIN_1
#define RADIO_DIO_1_GPIO_Port GPIOB
#define GPS_TX_Pin GPIO_PIN_10
#define GPS_TX_GPIO_Port GPIOB
#define GPS_RX_Pin GPIO_PIN_11
#define GPS_RX_GPIO_Port GPIOB
#define BTN1_Pin GPIO_PIN_12
#define BTN1_GPIO_Port GPIOB
#define RADIO_RESET_Pin GPIO_PIN_13
#define RADIO_RESET_GPIO_Port GPIOB
#define LIS3DH_INT1_PIN_Pin GPIO_PIN_14
#define LIS3DH_INT1_PIN_GPIO_Port GPIOB
#define LIS3DH_INT2_PIN_Pin GPIO_PIN_15
#define LIS3DH_INT2_PIN_GPIO_Port GPIOB
#define USB_TX_Pin GPIO_PIN_9
#define USB_TX_GPIO_Port GPIOA
#define USB_RX_Pin GPIO_PIN_10
#define USB_RX_GPIO_Port GPIOA
#define RADIO_DIO_0_Pin GPIO_PIN_11
#define RADIO_DIO_0_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_12
#define LED1_GPIO_Port GPIOA
#define GPS_POWER_ON_PIN_Pin GPIO_PIN_15
#define GPS_POWER_ON_PIN_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_4
#define LED2_GPIO_Port GPIOB
#define RADIO_RF_CRX_RX_Pin GPIO_PIN_6
#define RADIO_RF_CRX_RX_GPIO_Port GPIOB
#define RADIO_RF_CBT_HF_Pin GPIO_PIN_7
#define RADIO_RF_CBT_HF_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

extern unsigned char rxBuffer1[100], rxData1[2], rxBuffer3[100], rxData3[2];
extern volatile uint8_t rxIndex1, DataReady1, rxIndex3,DataReady3; 

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
