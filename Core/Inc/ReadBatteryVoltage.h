#pragma once

#include "main.h"

extern ADC_HandleTypeDef hadc2;

void readBatteryVoltageTask(void const* arg);
