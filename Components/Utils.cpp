/**
 ******************************************************************************
 * File Name          : Utils.cpp
 * Description        : Utility functions
 ******************************************************************************
*/
#include "Utils.hpp"

#include <cstring>

#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"
#include "main_avionics.hpp"
#include "SystemDefines.hpp"

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

/**
 * @brief Generates a CRC32 checksum for a given array of data using CRC Peripheral
 * @param data The data to generate the checksum for
 * @param size The size of the data array in uint8_t
 */
uint32_t Utils::getCRC32(uint8_t* data, uint32_t size)
{
	// Figure out the number of bytes to pad by
    uint8_t pad = 0;

	// If the buffer is not a multiple of 4 bytes, then we need to pad the buffer by the remaining bytes
    if(size % 4 == 0)
        pad = 4 - (size % 4);

    // Generate a buffer padded to uint32_t
    uint32_t buffer[(size + pad) / 4];
    uint8_t* tempPtr = (uint8_t*)(&buffer[0]);

    // Bytewise copy and pad
	memcpy(tempPtr, data, size);
	memset(tempPtr + size, 0, pad);

    // TODO: TEST THIS THING, also note there's a more efficient way (0-copy) that just involves loop accumulating 4x uint8_t's into 1x uint32_t's but this is more readable, ish
    // TODO: To be fair, G++ is very good at compiling memcpy though, so honestly other than instantaneous stack usage this may actually be more efficient
	SOAR_PRINT("Warning, HCRC is not tested!\n");

	// Calculate the CRC32
	return HAL_CRC_Calculate(SystemHandles::CRC_Handle, (uint32_t*)buffer, (size+pad)/4);
}

/**
 * @brief Converts a c string to a int32_t
 * @param str The string to convert, must be null terminated
 * @return The converted int32_t, or ERRVAL on an error
 */
int32_t Utils::stringToLong(const char* str)
{
    int32_t result = 0;
    const uint8_t size = (strlen(str) < 255) ? strlen(str) : 255;

    for (uint8_t i = 0; i < size; i++)
    {
        const uint8_t c = str[i];
        if (IsAsciiNum(c))
        {
            result *= 10;
            result += c - '0';
        }
        else
        {
            return ERRVAL;
        }
    }

    return result;

}
