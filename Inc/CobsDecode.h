#pragma once

#include "Data.h"

void cobs_dec(unsigned char *src, unsigned char len, unsigned char *dst);
void cobsDecodeTask(void const* arg);

extern UART_HandleTypeDef huart2;

void COBS_Process(CobsData* cobsData);
void callback2(UART_HandleTypeDef* huart);
