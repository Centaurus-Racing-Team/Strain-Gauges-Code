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
#define STR1_IN_Pin GPIO_PIN_0
#define STR1_IN_GPIO_Port GPIOA
#define STR2_IN_Pin GPIO_PIN_1
#define STR2_IN_GPIO_Port GPIOA
#define STR3_IN_Pin GPIO_PIN_2
#define STR3_IN_GPIO_Port GPIOA
#define STR4_IN_Pin GPIO_PIN_3
#define STR4_IN_GPIO_Port GPIOA
#define STR5_IN_Pin GPIO_PIN_4
#define STR5_IN_GPIO_Port GPIOA
#define STR6_IN_Pin GPIO_PIN_1
#define STR6_IN_GPIO_Port GPIOB
#define CS1_Pin GPIO_PIN_14
#define CS1_GPIO_Port GPIOB
#define CS2_Pin GPIO_PIN_9
#define CS2_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
