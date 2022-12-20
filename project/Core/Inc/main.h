/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
struct CAN_Msg_Count
{
	uint32_t TxMID_260;
	uint32_t TxMID_270;
	uint32_t TxMID_280;
	uint32_t TxMID_300;
	uint32_t TxMID_305;
};
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void set_MCU_IMU_data(void);
uint8_t is_sgt_CAN_busy();
void reset_CAN_msgs_counter(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USART_TX_IN_Pin GPIO_PIN_2
#define USART_TX_IN_GPIO_Port GPIOA
#define USART_RX_OUT_Pin GPIO_PIN_3
#define USART_RX_OUT_GPIO_Port GPIOA
#define ERROR_LED_Pin GPIO_PIN_10
#define ERROR_LED_GPIO_Port GPIOC
#define USART_RX_L_Pin GPIO_PIN_11
#define USART_RX_L_GPIO_Port GPIOC
#define CAN_RX_L_Pin GPIO_PIN_3
#define CAN_RX_L_GPIO_Port GPIOB
#define CAN_TX_L_Pin GPIO_PIN_4
#define CAN_TX_L_GPIO_Port GPIOB
#define CAN_RX_Pin GPIO_PIN_8
#define CAN_RX_GPIO_Port GPIOB
#define CAN_TX_Pin GPIO_PIN_9
#define CAN_TX_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
