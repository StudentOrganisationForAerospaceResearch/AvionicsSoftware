#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"
#include "Data.h"
#include "CobsDecode.h"

/**
 * Performs decoding using Consistent Overhead Byte Stuffing (COBS)
 * @param *src is the source pointer
 * @param len is the anticipated length of the destination array
 * @param *dst is the destination array
 */
void cobs_dec(unsigned char* src, unsigned char len, unsigned char* dst)
{
    unsigned char* end = src + len;

    while (src < end)
    {
        unsigned char i, c = *src++;

        for (i = 1; i < c; i++)
        {
            *dst++ = *src++;
        }

        if (c < 0xff)
        {
            *dst++ = 0;
        }
    }
}

void cobsDecodeTask(void const* arg)
{
    CobsData* cobsData = (CobsData*) arg;
    uint32_t prevWakeTime = osKernelSysTick();

    for (;;)
    {
        osDelayUntil(&prevWakeTime, 2);
        COBS_Process(cobsData);
    }
}

void COBS_Process(CobsData* cobsData)
{
    if (cobsData->parseData)
    {
        cobs_dec(cobsData->rxBuffer, 12, cobsData->dest);
        cobsData->rxIndex = 0;
        cobsData->parseData = 0;
    }

    HAL_UART_Receive_IT(&huart2, &cobsData->rxBuffer[cobsData->rxIndex], 1);

}
