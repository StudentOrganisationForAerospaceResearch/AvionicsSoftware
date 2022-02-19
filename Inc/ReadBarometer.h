#pragma once

#include "main.h"

extern SPI_HandleTypeDef hspi3;
extern UART_HandleTypeDef huart5;

void readBarometerTask(void const* arg);

