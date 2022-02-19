#pragma once

#include "main.h"

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart5;

void readAccelGyroMagnetismTask(void const* arg);
