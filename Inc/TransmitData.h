#pragma once

void transmitDataTask(void const* arg);

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;

extern int ventValveIsOpen;
extern int injectionValveIsOpen;
extern uint8_t launchSystemsRxChar;
