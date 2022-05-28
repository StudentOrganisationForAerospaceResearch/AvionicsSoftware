#pragma once

#include "main.h"
#include "Globals.h"

extern SPI_HandleTypeDef IMU_SPI;

void readAccelGyroMagnetismTask(void const* arg);
