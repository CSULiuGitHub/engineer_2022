/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OSC_IN_Pin GPIO_PIN_0
#define OSC_IN_GPIO_Port GPIOH
#define OSC_OUT_Pin GPIO_PIN_1
#define OSC_OUT_GPIO_Port GPIOH
#define LED3_Pin GPIO_PIN_2
#define LED3_GPIO_Port GPIOC
#define LED4_Pin GPIO_PIN_3
#define LED4_GPIO_Port GPIOC
#define KeyADC_Pin GPIO_PIN_1
#define KeyADC_GPIO_Port GPIOA
#define GRAB_FLOOR_Pin GPIO_PIN_5
#define GRAB_FLOOR_GPIO_Port GPIOA
#define RESCUE_Pin GPIO_PIN_6
#define RESCUE_GPIO_Port GPIOA
#define CARD_Pin GPIO_PIN_7
#define CARD_GPIO_Port GPIOA
#define GRAB_MODE_Pin GPIO_PIN_14
#define GRAB_MODE_GPIO_Port GPIOE
#define GRAB_STATUS_Pin GPIO_PIN_13
#define GRAB_STATUS_GPIO_Port GPIOE
#define CAN2_RX_Pin GPIO_PIN_12
#define CAN2_RX_GPIO_Port GPIOB
#define CAN2_TX_Pin GPIO_PIN_13
#define CAN2_TX_GPIO_Port GPIOB
#define JUDGE_TX_Pin GPIO_PIN_8
#define JUDGE_TX_GPIO_Port GPIOD
#define JUDGE_RX_Pin GPIO_PIN_9
#define JUDGE_RX_GPIO_Port GPIOD
#define LED1_Pin GPIO_PIN_10
#define LED1_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_11
#define LED2_GPIO_Port GPIOD
#define VISION_TX_Pin GPIO_PIN_6
#define VISION_TX_GPIO_Port GPIOC
#define VISION_RX_Pin GPIO_PIN_7
#define VISION_RX_GPIO_Port GPIOC
#define DR16_NC_Pin GPIO_PIN_9
#define DR16_NC_GPIO_Port GPIOA
#define DR16_RX_Pin GPIO_PIN_10
#define DR16_RX_GPIO_Port GPIOA
#define CAN1_RX_Pin GPIO_PIN_11
#define CAN1_RX_GPIO_Port GPIOA
#define CAN1_TX_Pin GPIO_PIN_12
#define CAN1_TX_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define IMU_NC_Pin GPIO_PIN_10
#define IMU_NC_GPIO_Port GPIOC
#define IMU_RX_Pin GPIO_PIN_11
#define IMU_RX_GPIO_Port GPIOC
#define UART_TX_Pin GPIO_PIN_5
#define UART_TX_GPIO_Port GPIOD
#define UART_RX_Pin GPIO_PIN_6
#define UART_RX_GPIO_Port GPIOD
#define OLED_SCL_Pin GPIO_PIN_6
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_7
#define OLED_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
