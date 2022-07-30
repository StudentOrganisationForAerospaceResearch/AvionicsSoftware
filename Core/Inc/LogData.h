#pragma once

#include "../../Drivers/w25qxx/w25qxx.h"
#include "Data.h"
#include "main.h"

extern UART_HandleTypeDef huart5;
extern SPI_HandleTypeDef hspi2;
extern w25qxx_t w25qxx;
extern CRC_HandleTypeDef hcrc;

extern uint8_t isOkayToLog;
extern uint8_t isErasing;
extern uint32_t currentSectorAddr;
extern uint32_t currentSectorOffset_B;

void initializeLogEntry(LogEntry* givenLog);
void logDataTask(void const* arg);
