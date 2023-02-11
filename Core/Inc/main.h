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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DMB_ABORT_Pin GPIO_PIN_14
#define DMB_ABORT_GPIO_Port GPIOC
#define DMB_ABORT_EXTI_IRQn EXTI15_10_IRQn
#define LAUNCH_Pin GPIO_PIN_15
#define LAUNCH_GPIO_Port GPIOC
#define LAUNCH_EXTI_IRQn EXTI15_10_IRQn
#define BATT_SENSE_Pin GPIO_PIN_0
#define BATT_SENSE_GPIO_Port GPIOC
#define AUX_2_Pin GPIO_PIN_2
#define AUX_2_GPIO_Port GPIOC
#define AUX_1_Pin GPIO_PIN_3
#define AUX_1_GPIO_Port GPIOC
#define GPS_TX_Pin GPIO_PIN_0
#define GPS_TX_GPIO_Port GPIOA
#define GPS_RX_Pin GPIO_PIN_1
#define GPS_RX_GPIO_Port GPIOA
#define RADIO_TX_Pin GPIO_PIN_2
#define RADIO_TX_GPIO_Port GPIOA
#define RADIO_RX_Pin GPIO_PIN_3
#define RADIO_RX_GPIO_Port GPIOA
#define IMU_XL_GY_CS_Pin GPIO_PIN_4
#define IMU_XL_GY_CS_GPIO_Port GPIOA
#define IMU_SPI_SCK_Pin GPIO_PIN_5
#define IMU_SPI_SCK_GPIO_Port GPIOA
#define IMU_SPI_MISO_Pin GPIO_PIN_6
#define IMU_SPI_MISO_GPIO_Port GPIOA
#define IMU_SPI_MOSI_Pin GPIO_PIN_7
#define IMU_SPI_MOSI_GPIO_Port GPIOA
#define IMU_MAG_CS_Pin GPIO_PIN_4
#define IMU_MAG_CS_GPIO_Port GPIOC
#define AV_PRESSURE_Pin GPIO_PIN_1
#define AV_PRESSURE_GPIO_Port GPIOB
#define PROP_TX_Pin GPIO_PIN_10
#define PROP_TX_GPIO_Port GPIOB
#define PROP_RX_Pin GPIO_PIN_11
#define PROP_RX_GPIO_Port GPIOB
#define RCU_DRIV_EN_Pin GPIO_PIN_12
#define RCU_DRIV_EN_GPIO_Port GPIOB
#define SPI_SCK_Pin GPIO_PIN_13
#define SPI_SCK_GPIO_Port GPIOB
#define SPI_MISO_Pin GPIO_PIN_14
#define SPI_MISO_GPIO_Port GPIOB
#define SPI_MOSI_Pin GPIO_PIN_15
#define SPI_MOSI_GPIO_Port GPIOB
#define LED_3_Pin GPIO_PIN_7
#define LED_3_GPIO_Port GPIOC
#define LED_2_Pin GPIO_PIN_8
#define LED_2_GPIO_Port GPIOC
#define LED_1_Pin GPIO_PIN_9
#define LED_1_GPIO_Port GPIOC
#define BAT_EN_Pin GPIO_PIN_8
#define BAT_EN_GPIO_Port GPIOA
#define RCU_TX_Pin GPIO_PIN_9
#define RCU_TX_GPIO_Port GPIOA
#define RCU_RX_Pin GPIO_PIN_10
#define RCU_RX_GPIO_Port GPIOA
#define SOL_CTRL_Pin GPIO_PIN_11
#define SOL_CTRL_GPIO_Port GPIOA
#define BUZZER_PWM_Pin GPIO_PIN_15
#define BUZZER_PWM_GPIO_Port GPIOA
#define BARO_SPI_SCK_Pin GPIO_PIN_10
#define BARO_SPI_SCK_GPIO_Port GPIOC
#define BARO_SPI_MISO_Pin GPIO_PIN_11
#define BARO_SPI_MISO_GPIO_Port GPIOC
#define DEBUG_TX_Pin GPIO_PIN_12
#define DEBUG_TX_GPIO_Port GPIOC
#define DEBUG_RX_Pin GPIO_PIN_2
#define DEBUG_RX_GPIO_Port GPIOD
#define BARO_CS_Pin GPIO_PIN_4
#define BARO_CS_GPIO_Port GPIOB
#define BARO_SPI_MOSI_Pin GPIO_PIN_5
#define BARO_SPI_MOSI_GPIO_Port GPIOB
#define SPI_MEM_WP_Pin GPIO_PIN_6
#define SPI_MEM_WP_GPIO_Port GPIOB
#define SPI_CS__Pin GPIO_PIN_7
#define SPI_CS__GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
