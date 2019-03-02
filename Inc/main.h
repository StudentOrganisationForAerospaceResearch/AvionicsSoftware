/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#define LAUNCH_Pin GPIO_PIN_3
#define LAUNCH_GPIO_Port GPIOC
#define GPS_UART_TX_Pin GPIO_PIN_0
#define GPS_UART_TX_GPIO_Port GPIOA
#define GPS_UART_RX_Pin GPIO_PIN_1
#define GPS_UART_RX_GPIO_Port GPIOA
#define LAUNCH_SYS_UART_TX_Pin GPIO_PIN_2
#define LAUNCH_SYS_UART_TX_GPIO_Port GPIOA
#define LAUNCH_SYS_UART_RX_Pin GPIO_PIN_3
#define LAUNCH_SYS_UART_RX_GPIO_Port GPIOA
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
#define LED1_Pin GPIO_PIN_5
#define LED1_GPIO_Port GPIOC
#define COMBUSTION_CHAMBER_ADC_Pin GPIO_PIN_0
#define COMBUSTION_CHAMBER_ADC_GPIO_Port GPIOB
#define OXIDIZER_TANK_ADC_Pin GPIO_PIN_1
#define OXIDIZER_TANK_ADC_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_2
#define LED2_GPIO_Port GPIOB
#define BARO_SPI_SCK_Pin GPIO_PIN_13
#define BARO_SPI_SCK_GPIO_Port GPIOB
#define BARO_SPI_MISO_Pin GPIO_PIN_14
#define BARO_SPI_MISO_GPIO_Port GPIOB
#define BARO_SPI_MOSI_Pin GPIO_PIN_15
#define BARO_SPI_MOSI_GPIO_Port GPIOB
#define BARO_CS_Pin GPIO_PIN_6
#define BARO_CS_GPIO_Port GPIOC
#define DROGUE_PARACHUTE_Pin GPIO_PIN_7
#define DROGUE_PARACHUTE_GPIO_Port GPIOC
#define MAIN_PARACHUTE_Pin GPIO_PIN_8
#define MAIN_PARACHUTE_GPIO_Port GPIOC
#define VENT_VALVE_Pin GPIO_PIN_11
#define VENT_VALVE_GPIO_Port GPIOA
#define INJECTION_VALVE_Pin GPIO_PIN_12
#define INJECTION_VALVE_GPIO_Port GPIOA
#define SD1_CS_Pin GPIO_PIN_15
#define SD1_CS_GPIO_Port GPIOA
#define SD_SPI_SCK_Pin GPIO_PIN_10
#define SD_SPI_SCK_GPIO_Port GPIOC
#define SD_SPI_MISO_Pin GPIO_PIN_11
#define SD_SPI_MISO_GPIO_Port GPIOC
#define SD_SPI_MOSI_Pin GPIO_PIN_12
#define SD_SPI_MOSI_GPIO_Port GPIOC
#define SD2_CS_Pin GPIO_PIN_2
#define SD2_CS_GPIO_Port GPIOD
#define FAN_CTRL_Pin GPIO_PIN_5
#define FAN_CTRL_GPIO_Port GPIOB
#define RADIO_UART_TX_Pin GPIO_PIN_6
#define RADIO_UART_TX_GPIO_Port GPIOB
#define RADIO_UART_RX_Pin GPIO_PIN_7
#define RADIO_UART_RX_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
