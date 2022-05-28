#pragma once

#include "main.h"
#include "Globals.h"

extern SPI_HandleTypeDef BARO_SPI;

void readBarometerTask(void const* arg);

