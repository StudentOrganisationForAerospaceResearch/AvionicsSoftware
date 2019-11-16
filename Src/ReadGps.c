#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "ReadGps.h"

#include "Data.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int READ_GPS_PERIOD = 500;

void readGpsTask(void const* arg)
{
    GpsData* data = (GpsData*) arg;
    uint32_t prevWakeTime = osKernelSysTick();

    HAL_UART_Receive_DMA(&huart4, (uint8_t*) &dma_rx_buffer, NMEA_MAX_LENGTH + 1);

    for (;;)
    {
        osDelayUntil(&prevWakeTime, READ_GPS_PERIOD);

        if (osMutexWait(data->mutex_, 0) != osOK)
        {
            continue;
        }

        if (data->parse == 1)
        {
            // Returns first token
            char* gps_item = strtok(data->buffer_, ",");
            uint8_t counter = 0;
            char direction;

            // Keep printing tokens while one of the
            // delimiters present in str[].
            while (gps_item != NULL)
            {
                switch (counter)
                {
                    case 1:
                    {
                        data->time_ = (uint32_t) atof(gps_item);
                        break;
                    }

                    case 2:
                    {
                        double latitude = (atof(gps_item));
                        data->latitude_.degrees_ = (int32_t) latitude / 100;
                        data->latitude_.minutes_ = (uint32_t) ((latitude - data->latitude_.degrees_ * 100) * 100000);
                        break;
                    }

                    case 3:		// Unit
                        direction = *gps_item;

                        if (direction == 'S')
                        {
                            data->latitude_.degrees_ *= -1;
                        }

                        break;

                    case 4:
                    {
                        double longitude = (atof(gps_item));
                        data->longitude_.degrees_ = (int32_t) longitude / 100;
                        data->longitude_.minutes_ = (uint32_t) ((longitude - data->longitude_.degrees_ * 100) * 100000);
                        break;
                    }

                    case 5:		// Unit
                        // If W, add a -
                        direction = *gps_item;

                        if (direction == 'W')
                        {
                            data->longitude_.degrees_ *= -1;
                        }

                        break;

                    case 9:
                    {
                        data->antennaAltitude_.altitude_ = (int32_t) (atof(gps_item) * 10);
                        break;
                    }

                    case 10:	// Unit
                        data->antennaAltitude_.unit_ = *gps_item;
                        break;

                    case 11:
                    {
                        data->geoidAltitude_.altitude_ = (int32_t) (atof(gps_item) * 10);
                        break;
                    }

                    case 12:	// Unit
                        data->geoidAltitude_.unit_ = *gps_item;
                        break;

                    default:
                        break;
                }

                counter++;
                gps_item = strtok(NULL, ",");
            }
        }

        data->parse = 0;
        osMutexRelease(data->mutex_);
    }
}
