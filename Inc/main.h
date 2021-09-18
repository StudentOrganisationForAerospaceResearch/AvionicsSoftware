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
#define MAIN_PARACHUTE_Pin GPIO_PIN_14
#define MAIN_PARACHUTE_GPIO_Port GPIOC
#define PMB_GPIO_1_Pin GPIO_PIN_15
#define PMB_GPIO_1_GPIO_Port GPIOC
#define BATT_SENSE_ADC_Pin GPIO_PIN_0
#define BATT_SENSE_ADC_GPIO_Port GPIOC
#define AUX2_Pin GPIO_PIN_2
#define AUX2_GPIO_Port GPIOC
#define AUX1_Pin GPIO_PIN_3
#define AUX1_GPIO_Port GPIOC
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
#define LED_3_Pin GPIO_PIN_14
#define LED_3_GPIO_Port GPIOB
#define LED_2_Pin GPIO_PIN_15
#define LED_2_GPIO_Port GPIOB
#define LED_1_Pin GPIO_PIN_6
#define LED_1_GPIO_Port GPIOC
#define LAUNCH_Pin GPIO_PIN_8
#define LAUNCH_GPIO_Port GPIOA
#define LAUNCH_SYS_UART_TX_Pin GPIO_PIN_9
#define LAUNCH_SYS_UART_TX_GPIO_Port GPIOA
#define LAUNCH_SYS_UART_RX_Pin GPIO_PIN_10
#define LAUNCH_SYS_UART_RX_GPIO_Port GPIOA
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
#define MEM_WP_Pin GPIO_PIN_7
#define MEM_WP_GPIO_Port GPIOB
#define EEPROM_I2C_SCL_Pin GPIO_PIN_8
#define EEPROM_I2C_SCL_GPIO_Port GPIOB
#define EEPROM_I2C_SDA_Pin GPIO_PIN_9
#define EEPROM_I2C_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
