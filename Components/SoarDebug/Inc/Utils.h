#pragma once

#include "SystemDefines.hpp"

uint16_t averageArray(uint16_t array[], int size);
void writeInt32ToArray(uint8_t* array, int startIndex, int32_t value);
void readUInt32FromUInt8Array(uint8_t* array, int startIndex, int32_t* value);
