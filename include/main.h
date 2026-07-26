/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define COMP_ADC_5_Pin GPIO_PIN_13
#define COMP_ADC_5_GPIO_Port GPIOC
#define COMP_ADC_6_Pin GPIO_PIN_14
#define COMP_ADC_6_GPIO_Port GPIOC
#define COMP_ADC_7_Pin GPIO_PIN_15
#define COMP_ADC_7_GPIO_Port GPIOC
#define COMP_ADC_8_Pin GPIO_PIN_0
#define COMP_ADC_8_GPIO_Port GPIOA
#define EXT_ADC_1_Pin GPIO_PIN_1
#define EXT_ADC_1_GPIO_Port GPIOA
#define EXT_ADC_2_Pin GPIO_PIN_2
#define EXT_ADC_2_GPIO_Port GPIOA
#define EXT_ADC_3_Pin GPIO_PIN_3
#define EXT_ADC_3_GPIO_Port GPIOA
#define EXT_ADC_4_Pin GPIO_PIN_4
#define EXT_ADC_4_GPIO_Port GPIOA
#define DRV1_CURRENT_Pin GPIO_PIN_5
#define DRV1_CURRENT_GPIO_Port GPIOA
#define DRV2_CURRENT_Pin GPIO_PIN_6
#define DRV2_CURRENT_GPIO_Port GPIOA
#define TEMP_SENS_1_Pin GPIO_PIN_7
#define TEMP_SENS_1_GPIO_Port GPIOA
#define TEMP_SENS_2_Pin GPIO_PIN_0
#define TEMP_SENS_2_GPIO_Port GPIOB
#define COMP_ADC_1_Pin GPIO_PIN_1
#define COMP_ADC_1_GPIO_Port GPIOB
#define COMP_ADC_2_Pin GPIO_PIN_2
#define COMP_ADC_2_GPIO_Port GPIOB
#define DRV2_EN_B_Pin GPIO_PIN_10
#define DRV2_EN_B_GPIO_Port GPIOB
#define EN_RELAY_1_Pin GPIO_PIN_11
#define EN_RELAY_1_GPIO_Port GPIOB
#define EN_RELAY_2_Pin GPIO_PIN_12
#define EN_RELAY_2_GPIO_Port GPIOB
#define EN_RELAY_3_Pin GPIO_PIN_13
#define EN_RELAY_3_GPIO_Port GPIOB
#define EN_RELAY_4_Pin GPIO_PIN_14
#define EN_RELAY_4_GPIO_Port GPIOB
#define EN_RELAY_5_Pin GPIO_PIN_15
#define EN_RELAY_5_GPIO_Port GPIOB
#define DRV1_EN_B_Pin GPIO_PIN_8
#define DRV1_EN_B_GPIO_Port GPIOA
#define DRV1_EN_A_Pin GPIO_PIN_10
#define DRV1_EN_A_GPIO_Port GPIOA
#define CAN_STB_Pin GPIO_PIN_15
#define CAN_STB_GPIO_Port GPIOA
#define COMP_ADC_3_Pin GPIO_PIN_3
#define COMP_ADC_3_GPIO_Port GPIOB
#define COMP_ADC_3_EXTI_IRQn EXTI3_IRQn
#define COMP_ADC_4_Pin GPIO_PIN_4
#define COMP_ADC_4_GPIO_Port GPIOB
#define COMP_ADC_4_EXTI_IRQn EXTI4_IRQn
#define DRV2_EN_A_Pin GPIO_PIN_5
#define DRV2_EN_A_GPIO_Port GPIOB
#define DRV1_IN_A_Pin GPIO_PIN_6
#define DRV1_IN_A_GPIO_Port GPIOB
#define DRV1_IN_B_Pin GPIO_PIN_7
#define DRV1_IN_B_GPIO_Port GPIOB
#define DRV2_IN_A_Pin GPIO_PIN_8
#define DRV2_IN_A_GPIO_Port GPIOB
#define DRV2_IN_B_Pin GPIO_PIN_9
#define DRV2_IN_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
