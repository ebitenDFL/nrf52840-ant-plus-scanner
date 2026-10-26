/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32u3xx_hal.h"

#include "stm32u3xx_nucleo.h"
#include <stdio.h>

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
void MX_USB_PCD_Init(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RCC_OSC32_IN_Pin GPIO_PIN_14
#define RCC_OSC32_IN_GPIO_Port GPIOC
#define RCC_OSC32_OUT_Pin GPIO_PIN_15
#define RCC_OSC32_OUT_GPIO_Port GPIOC
#define IMU1_CS_Pin GPIO_PIN_2
#define IMU1_CS_GPIO_Port GPIOF
#define IMU2_CS_Pin GPIO_PIN_4
#define IMU2_CS_GPIO_Port GPIOF
#define VSENSE_Pin GPIO_PIN_2
#define VSENSE_GPIO_Port GPIOC
#define VBUS_DET_Pin GPIO_PIN_12
#define VBUS_DET_GPIO_Port GPIOF
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define DEBUG_JTMS_SWDIO_Pin GPIO_PIN_13
#define DEBUG_JTMS_SWDIO_GPIO_Port GPIOA
#define DEBUG_JTCK_SWCLK_Pin GPIO_PIN_14
#define DEBUG_JTCK_SWCLK_GPIO_Port GPIOA
#define DEBUG_JTDI_Pin GPIO_PIN_15
#define DEBUG_JTDI_GPIO_Port GPIOA
#define FDCAN_STBY_Pin GPIO_PIN_7
#define FDCAN_STBY_GPIO_Port GPIOD
#define DEBUG_JTDO_SWO_Pin GPIO_PIN_3
#define DEBUG_JTDO_SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
