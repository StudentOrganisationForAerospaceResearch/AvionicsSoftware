/**
  ******************************************************************************
  * File Name          : TransmitData.h
  ******************************************************************************
*/

#pragma once

/* Includes ------------------------------------------------------------------*/

/* Macros --------------------------------------------------------------------*/

/* Structs -------------------------------------------------------------------*/

/* Externs -------------------------------------------------------------------*/
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;

extern int injectionValveIsOpen;
extern int lowerVentValveIsOpen;
extern uint8_t launchSystemsRxChar;

/* Prototypes ----------------------------------------------------------------*/
void transmitDataTask(void const* arg);
