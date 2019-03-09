/**
 * **********************************************************
 * File Name        : decoder.c
 * ***********************************************************
 */
/* Externs --------------------------------------------------*/
#include "CobsEncode.h"

unsigned int frameData(unsigned char *dataToEncode, unsigned long length, unsigned char *frameData)
{
    unsigned int lengthOfFramedData = stuffData(dataToEncode, length, frameData);
    //adds the delimiter byte at the end
    frameData[lengthOfFramedData++] = 0x00;
    return lengthOfFramedData;
}

unsigned int stuffData(unsigned char *dataToEncode, unsigned long length, unsigned char *encodedData)
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


