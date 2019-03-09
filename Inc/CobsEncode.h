/**
************************************************************
* File Name     : encoder.h
************************************************************
*/

/* Includes ------------------------------------------------*/
#include <stdio.h>
/* Macros --------------------------------------------------*/
#define START (0xFFFF)
#define XOR_OUT (0xFFFF)
#define FINISH_BLOCK(X)           \
    {                             \
        *code_ptr = (X);          \
        code_ptr = encodedData++; \
        code = 0x01;              \
    }

/* Prototypes -----------------------------------------------*/
/**
 * Performs data framing through consistent overhead byte stuffing (COBS)
 * @param *dataToEncode is the source pointer that holds the char array waiting to be encoded
 * @param len is the length of the source (*dataToEncode) array
 * @param *frameData is the destination pointer
 */
unsigned int frameData(unsigned char *dataToEncode, unsigned long length, unsigned char *frameData);
/**
 * Performs the actual data stuffing on the arrays
 * @param *dataToEncode is the source pointer
 * @param length is the length of the *encodedData array
 * @param *encodedData is the size of the destination array
 * @return returns the length of encoded data
 */
unsigned int stuffData(unsigned char *dataToEncode, unsigned long length, unsigned char *encodedData);
