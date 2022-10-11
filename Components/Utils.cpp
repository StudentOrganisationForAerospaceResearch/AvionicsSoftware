/**
 ******************************************************************************
 * File Name          : Utils.cpp
 * Description        : Utility functions
 ******************************************************************************
*/
#include "Utils.hpp"

#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

/**
 * @brief Calculates the average from a list of unsigned shorts
 * @param array: The array of unsigned shorts to average
 * @param size: The size of the array
 * @return Returns the average as a uint16_t
 */
uint16_t Utils::averageArray(uint16_t array[], int size)
{
    uint16_t sum = 0;

    for (int i = 0; i < size; i++)
    {
        sum += array[i];
    }

    return (sum / size);
}

/**
 * @brief converts an int32 to a uint8_t array
 * right shift to put bytes in LSB slots, & with 0x00ff
 * @param array: The array to store the bytes in
 * @param startIndex: The index to start storing the bytes at
 * @param value: The int32 to convert
 */
void Utils::writeInt32ToArray(uint8_t* array, int startIndex, int32_t value)
{
    array[startIndex + 0] = (value >> 24) & 0xFF;
    array[startIndex + 1] = (value >> 16) & 0xFF;
    array[startIndex + 2] = (value >> 8) & 0xFF;
    array[startIndex + 3] = value & 0xFF;
}

/**
 * @brief converts a uint8_t* read from EEPROM to a uint32_t
 * @param array, the array read from the EEPROM (datRead)
 * @param startIndex, where the data field starts in the array
 * @param value, pointer to the data field that should be updated
 */
void Utils::readUInt32FromUInt8Array(uint8_t* array, int startIndex, int32_t* value)
{
    uint32_t temp = 0;
    temp += (array[startIndex + 0] << 24); // eeprom reads little or big endian?
    temp += (array[startIndex + 1] << 16);
    temp += (array[startIndex + 2] << 8);
    temp += (array[startIndex + 3]);
    *value = temp;
}
