#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "ReadGps.h"

#include "Data.h"
#include "main.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int READ_GPS_PERIOD = 250;

void readGpsTask(void const* arg)
{
    GpsData* data = (GpsData*) arg;
    uint32_t prevWakeTime = osKernelSysTick();


    /** Used to program GPS  TODO: separate from reading thread
   	 * send a UBX ConfigValSet message to set the RAM of the gps receiver
   	 * key value pair will be CFG-NMEA NMEA protocol configuration
   	 * the CFG-NMEA config table tells us it's type L (1 byte for the value)
   	 * */
   //    char configGpsBuffer[17] = {  0xb5, 0x62,					// header
   //   								0x06, 0x8a,					// class ID
   //       							0x0a,						// length
   //       							0x00, 0x01, 0x00, 0x00,     // payload head: 0 - layer - 0 - 0
   //   								0x30, 0x21, 0x00, 0x01,		// key
   //   								0x01, 0xf4,					// U2 value
   //   								0xe2, 0xfb 					// checksum
   //       							};
   //
   //    HAL_StatusTypeDef configstatus = HAL_UART_Transmit(&huart4, (uint8_t*) &configGpsBuffer, 17, 500);

    HAL_UART_Receive_DMA(&huart4, (uint8_t*) &dma_rx_buffer, NMEA_MAX_LENGTH + 1);

    for (;;)
    {
        osDelayUntil(&prevWakeTime, READ_GPS_PERIOD);

        if (osMutexWait(data->mutex_, 0) != osOK)
        {
            continue;
        }

        if (data->parseFlag_ == 1)
        {
			// Returns the first token
			char* gps_item = &data->buffer_[0];
			uint8_t item_len = 0;
			uint8_t counter = 0;
			uint8_t done = 0;
			char direction;
//			HAL_GPIO_TogglePin(LED_3_GPIO_Port, LED_3_Pin);

			// Keeps printing tokens while one of the delimiters present in gps_item
			do
			{
				for (uint8_t i = 0; i < NMEA_MAX_LENGTH + 1; i++) {
					if (gps_item[i] == ',') {
						gps_item[i] = 0;
						item_len = i;
						break;
					} else if (gps_item[i] == 0) {
						done = 1;
						break;
					}
				}

				if (gps_item[0] != 0) {
					switch (counter)
					{
						// case 0 is when gps_item is "$GPGGA"
						case 1:
						{
							data->time_ = (uint32_t) (atof(gps_item) * 100); // HHMMSS.SS format. Time is multiplied by 100.
							break;
						}

						case 2:
						{
							double latitude = (atof(gps_item)); // DDMM.MMMMMM
							data->latitude_.degrees_ = (int32_t) latitude / 100; // First 2 numbers are the latitude degrees
							data->latitude_.minutes_ = (int32_t) ((latitude - data->latitude_.degrees_ * 100) * 100000); // Latitude minutes is multplied by 100000
							break;
						}

						case 3:	// Latitude direction
						{
							direction = *gps_item;

							// N is represented as a positive value
							// S is represented as a negative value
							if (direction == 'S')
							{
								data->latitude_.degrees_ *= -1;
								data->latitude_.minutes_ *= -1;
							}

							break;
						}

						case 4:
						{
							double longitude = (atof(gps_item)); // DDMM.MMMMMM
							data->longitude_.degrees_ = (int32_t) longitude / 100; // First 2 numbers are the longitude degrees
							data->longitude_.minutes_ = (int32_t) ((longitude - data->longitude_.degrees_ * 100) * 100000); // Longitude minutes is multplied by 100000
							break;
						}

						case 5: // Longitude direction
						{
							direction = *gps_item;

							// E is represented as a positive value
							// W is represented as a negative value
							if (direction == 'W')
							{
								data->longitude_.degrees_ *= -1;
								data->longitude_.minutes_ *= -1;
							}

							break;
						}

						case 9:
						{
							data->antennaAltitude_.altitude_ = (int32_t) (atof(gps_item) * 10); // Antenna altitude is multiplied by 10
							break;
						}

						case 10: // Antenna altitude unit
						{
							data->antennaAltitude_.unit_ = *gps_item;
							break;
						}

						case 11:
						{
							data->geoidAltitude_.altitude_ = (int32_t) (atof(gps_item) * 10); // Geoid altitude is multiplied by 10
							break;
						}

						case 12: // Geoid altitude unit
						{
							data->geoidAltitude_.unit_ = *gps_item;
							break;
						}

						default:
							break;
					}
				}

				counter++;
				gps_item = &gps_item[item_len + 1];
			} while (done == 0);
		}

        // Subtract geoid altitude from antenna altitude to get Height Above Ellipsoid (HAE)
        data->totalAltitude_.altitude_ = data->antennaAltitude_.altitude_ - data->geoidAltitude_.altitude_;
        data->totalAltitude_.unit_ = data->antennaAltitude_.unit_;

        data->parseFlag_ = 0;
        memset(dma_rx_buffer, 0, NMEA_MAX_LENGTH+1);
        osMutexRelease(data->mutex_);
    }
}
