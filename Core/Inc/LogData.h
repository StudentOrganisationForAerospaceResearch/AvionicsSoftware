#pragma once

#include "main.h"

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart5;

void logDataTask(void const* arg);
