/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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

/*void initSEG (void);
void all_on_SEG (void);*/

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ROT_A_2_Pin GPIO_PIN_13
#define ROT_A_2_GPIO_Port GPIOC
#define ROT_B_2_Pin GPIO_PIN_14
#define ROT_B_2_GPIO_Port GPIOC
#define ROT_SW_2_Pin GPIO_PIN_15
#define ROT_SW_2_GPIO_Port GPIOC
#define ROT_SW_1_Pin GPIO_PIN_0
#define ROT_SW_1_GPIO_Port GPIOB
#define ROT_B_1_Pin GPIO_PIN_1
#define ROT_B_1_GPIO_Port GPIOB
#define ROT_A_1_Pin GPIO_PIN_2
#define ROT_A_1_GPIO_Port GPIOB
#define DIGIT8_Pin GPIO_PIN_12
#define DIGIT8_GPIO_Port GPIOB
#define DIGIT7_Pin GPIO_PIN_13
#define DIGIT7_GPIO_Port GPIOB
#define DIGIT6_Pin GPIO_PIN_14
#define DIGIT6_GPIO_Port GPIOB
#define DIGIT5_Pin GPIO_PIN_15
#define DIGIT5_GPIO_Port GPIOB
#define SEG_A_Pin GPIO_PIN_15
#define SEG_A_GPIO_Port GPIOA
#define SEG_B_Pin GPIO_PIN_10
#define SEG_B_GPIO_Port GPIOC
#define SEG_C_Pin GPIO_PIN_11
#define SEG_C_GPIO_Port GPIOC
#define SEG_D_Pin GPIO_PIN_12
#define SEG_D_GPIO_Port GPIOC
#define SEG_E_Pin GPIO_PIN_2
#define SEG_E_GPIO_Port GPIOD
#define SEG_F_Pin GPIO_PIN_3
#define SEG_F_GPIO_Port GPIOB
#define SEG_G_Pin GPIO_PIN_4
#define SEG_G_GPIO_Port GPIOB
#define SEG_DP_Pin GPIO_PIN_5
#define SEG_DP_GPIO_Port GPIOB
#define DIGIT4_Pin GPIO_PIN_6
#define DIGIT4_GPIO_Port GPIOB
#define DIGIT3_Pin GPIO_PIN_7
#define DIGIT3_GPIO_Port GPIOB
#define DIGIT2_Pin GPIO_PIN_8
#define DIGIT2_GPIO_Port GPIOB
#define DIGIT1_Pin GPIO_PIN_9
#define DIGIT1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
