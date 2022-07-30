/**
 ******************************************************************************
 * File Name          : Main.hpp
 * Description        : Header file for Main.cpp, acts as an interface between
 *  STM32CubeIDE and our application.
 ******************************************************************************
*/
#ifndef AVIONICS_INCLUDE_SOAR_MAIN_H
#define AVIONICS_INCLUDE_SOAR_MAIN_H
#include "stm32f4xx_hal.h"

/* Interface Functions ------------------------------------------------------------------*/
/* These functions act as our program's 'main' and any functions inside CubeIDE's main --*/
void run_main();
void run_StartDefaultTask();

/* Global Functions ------------------------------------------------------------------*/
void soar_assert_debug(bool condition, uint8_t* file, uint16_t line, const char* str = nullptr, ...);


/* System Handles ------------------------------------------------------------------*/
/* This should be the only place externs are allowed -------------------------------*/
namespace SystemHandles
{
    //UART Handles
    extern UART_HandleTypeDef huart1;   // UART1 - Launch Systems  ... Confirm
    extern UART_HandleTypeDef huart2;   // UART2 - Logging (Radio)
    extern UART_HandleTypeDef huart4;   // UART4 - GPS
    extern UART_HandleTypeDef huart5;   // UART5 - Debug

    //ADC Handles
	extern ADC_HandleTypeDef hadc1;      // ADC1 - Combustion Chamber ADC
    extern ADC_HandleTypeDef hadc2;      // ADC2 - Battery

    //I2C Handles
    extern I2C_HandleTypeDef hi2c1;      // I2C1 -- EEPROM (? - Do we still have an I2C EEPROM)

	//SPI Handles
    extern SPI_HandleTypeDef hspi1;      // SPI1 - 
    extern SPI_HandleTypeDef hspi3;      // SPI3 - Barometer MOSI/MISO/CLK

    //CRC Handles
	extern CRC_HandleTypeDef hcrc;       // CRC -
}

#endif /* AVIONICS_INCLUDE_SOAR_MAIN_H */