/* USER CODE BEGIN Header */
/* USER CODE END Header */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#define motor_i1_Pin GPIO_PIN_9
#define motor_i1_GPIO_Port GPIOA
#define motor_i2_Pin GPIO_PIN_10
#define motor_i2_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif
