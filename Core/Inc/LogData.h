#pragma once

#include "main.h"
#include "../../Drivers/w25qxx/w25qxx.h"

#define SPIMODE 1

//#ifdef I2CMODE
//extern I2C_HandleTypeDef hi2c1;
//#endif

extern UART_HandleTypeDef huart5;
extern SPI_HandleTypeDef hspi2;
extern w25qxx_t w25qxx;

HAL_StatusTypeDef writeLogEntryToEEPROM(uint16_t memAddress, LogEntry* givenLog);
void readLogEntryFromEEPROM(uint16_t memAddress, LogEntry* givenLog);
void initializeLogEntry(LogEntry* givenLog);
void logDataTask(void const* arg);
