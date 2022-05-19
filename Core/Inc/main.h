/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PMB_GPIO_1_Pin GPIO_PIN_14
#define PMB_GPIO_1_GPIO_Port GPIOC
#define BATT_SENSE_ADC_Pin GPIO_PIN_0
#define BATT_SENSE_ADC_GPIO_Port GPIOC
#define AUX_2_Pin GPIO_PIN_2
#define AUX_2_GPIO_Port GPIOC
#define AUX_1_Pin GPIO_PIN_3
#define AUX_1_GPIO_Port GPIOC
#define GPS_UART_TX_Pin GPIO_PIN_0
#define GPS_UART_TX_GPIO_Port GPIOA
#define GPS_UART_RX_Pin GPIO_PIN_1
#define GPS_UART_RX_GPIO_Port GPIOA
#define RADIO_UART_TX_Pin GPIO_PIN_2
#define RADIO_UART_TX_GPIO_Port GPIOA
#define RADIO_UART_RX_Pin GPIO_PIN_3
#define RADIO_UART_RX_GPIO_Port GPIOA
#define IMU_CS_Pin GPIO_PIN_4
#define IMU_CS_GPIO_Port GPIOA
#define IMU_SPI_SCK_Pin GPIO_PIN_5
#define IMU_SPI_SCK_GPIO_Port GPIOA
#define IMU_SPI_MISO_Pin GPIO_PIN_6
#define IMU_SPI_MISO_GPIO_Port GPIOA
#define IMU_SPI_MOSI_Pin GPIO_PIN_7
#define IMU_SPI_MOSI_GPIO_Port GPIOA
#define MAG_CS_Pin GPIO_PIN_4
#define MAG_CS_GPIO_Port GPIOC
#define COMBUSTION_CHAMBER_ADC_Pin GPIO_PIN_5
#define COMBUSTION_CHAMBER_ADC_GPIO_Port GPIOC
#define OXIDIZER_TANK_ADC_Pin GPIO_PIN_0
#define OXIDIZER_TANK_ADC_GPIO_Port GPIOB
#define PROPULSION_3_VALVE_ADC_Pin GPIO_PIN_1
#define PROPULSION_3_VALVE_ADC_GPIO_Port GPIOB
#define LOWER_VENT_VALVE_Pin GPIO_PIN_2
#define LOWER_VENT_VALVE_GPIO_Port GPIOB
#define INJECTION_VALVE_Pin GPIO_PIN_10
#define INJECTION_VALVE_GPIO_Port GPIOB
#define PROPULSION_3_VALVE_Pin GPIO_PIN_11
#define PROPULSION_3_VALVE_GPIO_Port GPIOB
#define SPI_FLASH_SCK_Pin GPIO_PIN_13
#define SPI_FLASH_SCK_GPIO_Port GPIOB
#define SPI_FLASH_MISO_Pin GPIO_PIN_14
#define SPI_FLASH_MISO_GPIO_Port GPIOB
#define SPI_FLASH_MOSI_Pin GPIO_PIN_15
#define SPI_FLASH_MOSI_GPIO_Port GPIOB
#define KLB_CONTROL_Pin GPIO_PIN_6
#define KLB_CONTROL_GPIO_Port GPIOC
#define LED_3_Pin GPIO_PIN_7
#define LED_3_GPIO_Port GPIOC
#define LED_2_Pin GPIO_PIN_8
#define LED_2_GPIO_Port GPIOC
#define LED_1_Pin GPIO_PIN_9
#define LED_1_GPIO_Port GPIOC
#define LAUNCH_Pin GPIO_PIN_8
#define LAUNCH_GPIO_Port GPIOA
#define BUZZER_PWM_Pin GPIO_PIN_15
#define BUZZER_PWM_GPIO_Port GPIOA
#define BARO_SPI_SCK_Pin GPIO_PIN_10
#define BARO_SPI_SCK_GPIO_Port GPIOC
#define BARO_SPI_MISO_Pin GPIO_PIN_11
#define BARO_SPI_MISO_GPIO_Port GPIOC
#define DEBUG_UART_TX_Pin GPIO_PIN_12
#define DEBUG_UART_TX_GPIO_Port GPIOC
#define DEBUG_UART_RX_Pin GPIO_PIN_2
#define DEBUG_UART_RX_GPIO_Port GPIOD
#define BARO_CS_Pin GPIO_PIN_4
#define BARO_CS_GPIO_Port GPIOB
#define BARO_SPI_MOSI_Pin GPIO_PIN_5
#define BARO_SPI_MOSI_GPIO_Port GPIOB
#define MEM_WP_Pin GPIO_PIN_6
#define MEM_WP_GPIO_Port GPIOB
#define SPI_FLASH_CS_Pin GPIO_PIN_7
#define SPI_FLASH_CS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
