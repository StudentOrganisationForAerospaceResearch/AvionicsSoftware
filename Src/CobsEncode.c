/**
 * **********************************************************
 * File Name        : decoder.c
 * ***********************************************************
 */
/* Externs --------------------------------------------------*/
#include "CobsEncode.h"

/**
 * Adds a delimiter byte at the end of the encoded data done by stuffData and calls it as well
 * @param *dataToEncode is the source pointer
 * @param length is the anticipated length of the destination array (frameData)
 * @param *frameData is the destination array
 */
unsigned int frameData(uint8_t *dataToEncode, unsigned long length, uint8_t *frameData)
{
    unsigned int lengthOfFramedData = stuffData(dataToEncode, length, frameData);
    //adds the delimiter byte at the end
    frameData[lengthOfFramedData++] = 0x00;
    return lengthOfFramedData;
}

/**
 * Performs encoding using Consistent Overhead Byte Stuffing (COBS)
 * @param *dataToEncode is the source pointer
 * @param length is the anticipated length of the destination array (frameData)
 * @param *frameData is the destination array
 */
unsigned int stuffData(uint8_t *dataToEncode, unsigned long length, uint8_t *encodedData)
{
    unsigned int lengthOfEncodedData = length + 1;
    unsigned char *end = dataToEncode + length;
    unsigned char *code_ptr = encodedData++;
    unsigned char code = 0x01;

    while (dataToEncode < end)
    {
        if (*dataToEncode == 0)
        {
            FINISH_BLOCK(code);
        }
        else
        {
            *encodedData++ = *dataToEncode;
            code++;

            if (code == 0xFF)
            {
                FINISH_BLOCK(code);
            }
        }

        dataToEncode++;
    }

    FINISH_BLOCK(code);
    return lengthOfEncodedData;
}


