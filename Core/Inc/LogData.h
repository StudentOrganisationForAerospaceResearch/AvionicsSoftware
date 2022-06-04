#pragma once

#include "main.h"
#include "Data.h"
#include "../../Drivers/w25qxx/w25qxx.h"

#define SPIMODE 1

//#ifdef I2CMODE
//extern I2C_HandleTypeDef hi2c1;
//#endif

extern UART_HandleTypeDef huart5;
extern SPI_HandleTypeDef hspi2;
extern w25qxx_t w25qxx;
extern CRC_HandleTypeDef hcrc;

extern uint8_t isOkayToLog;
extern uint32_t currentSectorAddr;
extern uint32_t currentSectorOffset_B;

void initializeLogEntry(LogEntry* givenLog);
void logDataTask(void const* arg);
