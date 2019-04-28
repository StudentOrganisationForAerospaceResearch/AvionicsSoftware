#pragma once

void transmitDataTask(void const* arg);

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;

extern int upperVentValveIsOpen;
extern int injectionValveIsOpen;
extern int lowerVentValveIsOpen;
extern uint8_t launchSystemsRxChar;
