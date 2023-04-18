/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

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
#define MAG_INT_Pin GPIO_PIN_0
#define MAG_INT_GPIO_Port GPIOA
#define MAG_INT_EXTI_IRQn EXTI0_IRQn
#define MAG_INT_TRG_Pin GPIO_PIN_2
#define MAG_INT_TRG_GPIO_Port GPIOA
#define IMU3_INT1_Pin GPIO_PIN_3
#define IMU3_INT1_GPIO_Port GPIOA
#define IMU3_INT1_EXTI_IRQn EXTI3_IRQn
#define IMU3_INT2_Pin GPIO_PIN_4
#define IMU3_INT2_GPIO_Port GPIOA
#define IMU3_INT2_EXTI_IRQn EXTI4_IRQn
#define IMU1_CS_Pin GPIO_PIN_0
#define IMU1_CS_GPIO_Port GPIOB
#define IMU2_CS_Pin GPIO_PIN_1
#define IMU2_CS_GPIO_Port GPIOB
#define IMU3_CS_Pin GPIO_PIN_2
#define IMU3_CS_GPIO_Port GPIOB
#define FLASH_CS_Pin GPIO_PIN_13
#define FLASH_CS_GPIO_Port GPIOB
#define IMU2_INT1_Pin GPIO_PIN_9
#define IMU2_INT1_GPIO_Port GPIOA
#define IMU2_INT1_EXTI_IRQn EXTI9_5_IRQn
#define IMU2_INT2_Pin GPIO_PIN_10
#define IMU2_INT2_GPIO_Port GPIOA
#define IMU2_INT2_EXTI_IRQn EXTI15_10_IRQn
#define IMU1_INT1_Pin GPIO_PIN_11
#define IMU1_INT1_GPIO_Port GPIOA
#define IMU1_INT1_EXTI_IRQn EXTI15_10_IRQn
#define IMU1_INT2_Pin GPIO_PIN_12
#define IMU1_INT2_GPIO_Port GPIOA
#define IMU1_INT2_EXTI_IRQn EXTI15_10_IRQn
#define XBEE_CS_Pin GPIO_PIN_15
#define XBEE_CS_GPIO_Port GPIOA
#define SPI3_ATTN_Pin GPIO_PIN_7
#define SPI3_ATTN_GPIO_Port GPIOB
#define SPI3_ATTN_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
