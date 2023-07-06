/**
  ******************************************************************************
  * File Name          : GPSTask.cpp
  *
  * Source Info           : Based on Andromeda V3.31 Implementation
  *                         Andromeda_V3.31_Legacy/Core/Src/ReadGPS.c
  *
  * Description        : This file contains functions and constants to read
  *                      NMEA messages from the GPS module and parse them into
  *                      our telemetry data structure.
  ******************************************************************************
*/
#include "GPSTask.hpp"
#include "SystemDefines.hpp"
#include <stdlib.h>
#include <cstring>

/**
 * @brief Default constructor, sets up storage for member variables
 */
GPSTask::GPSTask() : Task(TASK_GPS_QUEUE_DEPTH_OBJS)
{
    data = (GpsData*)soar_malloc(sizeof(GpsData));
}

/**
 * @brief Initialize the GPS Task with the RTOS
 */
void GPSTask::InitTask()
{
    // Make sure the task is not already initialized
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize GPS task twice");
    
    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)GPSTask::RunTask,
            (const char*)"GPSTask",
            (uint16_t)TASK_GPS_STACK_DEPTH_WORDS,
            (void*)this,
            (UBaseType_t)TASK_GPS_PRIORITY,
            (TaskHandle_t*)&rtTaskHandle);
    
    SOAR_ASSERT(rtValue == pdPASS, "GPSTask::InitTask() - xTaskCreate() failed");
}

/**
 * @brief Handle DMA RxCplt interrupt
 * @source Andromeda_V3.31_Legacy/Core/Src/main.c @ 1036
 */
void GPSTask::HandleGPSRxComplete()
{
    static char rx_buffer[NMEA_MAX_LENGTH + 1];
    static int rx_index = 0;
    static int gpggaDetected = 0;
    const char message[7] = "$GPGGA";

    for (int i = 0; i < NMEA_MAX_LENGTH + 1; i++)
    {
        char rx = gpsTaskRxBuffer[i]; // Read 1 character

        if ((rx == '\r') || (rx == '\n')) // End of line character has been reached
        {
            if (rx_index != 0 && rx_buffer[0] == '$') // Check that buffer has content and that the message is valid
            {
                rx_buffer[rx_index++] = 0;

                if (gpsDataMutex.Lock(0))
                {
                    memcpy(&data->buffer_, &rx_buffer, rx_index); // Copy to gps data buffer from rx_buffer

                    // Notify the gps task that we have data available
                    Command cm(DATA_COMMAND, EVENT_GPS_RX_PARSE_READY);
                    qEvtQueue->SendFromISR(cm);

                    gpsDataMutex.Unlock();
                }

                // Reset back to initial values
                rx_index = 0;
                gpggaDetected = 0;
            }
        }
        else
        {
            if ((rx == '$') || (rx_index == NMEA_MAX_LENGTH)) // If start character received or end of rx buffer reached
            {
                // Reset back to initial values
                rx_index = 0;
                gpggaDetected = 0;
                rx_buffer[rx_index++] = rx;
            }
            else if (gpggaDetected == 0)
            {
                if (rx_index >= 6) // If the first 6 characters follow $GPGGA, set gpggaDetected to 1
                {
                    gpggaDetected = 1;
                    rx_buffer[rx_index++] = rx; // Contents of the rx_buffer will be $GPGGA at this point
                }
                else if (rx == message[rx_index]) // Check if the first 6 characters follow $GPGGA
                {
                    rx_buffer[rx_index++] = rx;
                }
                else
                {
                    // If any of the first 6 characters does not follow $GPGGA, reset to initial values
                    rx_index = 0;
                    gpggaDetected = 0;
                }
            }
            else
            {
                rx_buffer[rx_index++] = rx; // Copy received characters to rx_buffer
            }
        }
    }

    ReceiveData(); // Receive more data
}

/**
 * @brief Instance Run loop for GPS Task, runs on scheduler start as long as the task is initialized.
 * @param pvParams RTOS Passed void parameters, contains a pointer to the object instance, should not be used
 */
void GPSTask::Run(void * pvParams)
{
    //Setup the GPS
	ReceiveData();

    //Task run loop
    while(1) {
        Command cm;

        //Wait forever for a command
        qEvtQueue->ReceiveWait(cm);

        //Process the command
        HandleCommand(cm);
    }
}

/**
 * @brief Handles commands sent to the GPS Task
 */
void GPSTask::HandleCommand(Command& cm)
{
    //Switch for the GLOBAL_COMMAND
    switch (cm.GetCommand()) {
    case REQUEST_COMMAND: {
        HandleRequestCommand(cm.GetTaskCommand());
        break;
    }
    case TASK_SPECIFIC_COMMAND: {
        break;
    }
    case DATA_COMMAND: {
        if (cm.GetTaskCommand() == EVENT_GPS_RX_PARSE_READY)
            ParseGpsData();
        break;
    }
    default:
        SOAR_PRINT("GPSTask - Received Unsupported Command {%d}\n", cm.GetCommand());
        break;
    }

    //No matter what we happens, we must reset allocated data
    cm.Reset();
}

/**
 * @brief Handles requests sent to the GPS Task
 */
void GPSTask::HandleRequestCommand(uint16_t taskCommand)
{
    //Switch for task specific command within DATA_COMMAND
    switch (taskCommand) {
    case GPS_REQUEST_SAMPLE_INVALID:
        SOAR_PRINT("GPSTask - Received Sample Request (unsupported)\n");
        break;
    case GPS_REQUEST_TRANSMIT:
        SOAR_PRINT("Stubbed: GPS task transmit not implemented\n");
        break;
    case GPS_REQUEST_DEBUG:
        SOAR_PRINT("\t-- GPS Data --\n");
        SOAR_PRINT(" Time : %d\n", data->time_);
        SOAR_PRINT(" Latitude  (deg, min) : (%d, %d)\n", data->latitude_.degrees_, data->latitude_.minutes_);
        SOAR_PRINT(" Longitude (deg, min) : (%d, %d)\n", data->longitude_.degrees_, data->longitude_.minutes_);
        SOAR_PRINT(" Altitude   (N, unit) : (%d, %c)\n", data->antennaAltitude_.altitude_, data->antennaAltitude_.unit_);
        SOAR_PRINT(" Altitude   (N, unit) : (%d, %c)\n", data->geoidAltitude_.altitude_, data->geoidAltitude_.unit_);
        SOAR_PRINT(" Altitude   (N, unit) : (%d, %c)\n", data->totalAltitude_.altitude_, data->totalAltitude_.unit_);
        break;
    default:
        SOAR_PRINT("GPSTask - Received Unsupported REQUEST_COMMAND {%d}\n", taskCommand);
        break;
    }
}

/**
 * @brief Triggers HAL to receive GPS data via DMA
 * @return Status flag
 */
bool GPSTask::ReceiveData()
{
    HAL_UART_Receive_DMA(SystemHandles::UART_GPS, (uint8_t*)&gpsTaskRxBuffer, GPS_TASK_RX_BUFFER_SIZE);
    return true;
}

/**
 * @brief Parse GPS Data
 * @source Andromeda_V3.31_Legacy/Core/Src/ReadGPS.c @ 34
 */
void GPSTask::ParseGpsData()
{
    // Try to get the mutex, if we can't, don't parse
    if(!gpsDataMutex.Lock(READ_GPS_PERIOD_MS))
    {
        SOAR_PRINT("GPS Warning - Could not acquire mutex for parsing!\n");
    }

    // Vars
    char* gps_item = &data->buffer_[0];
    uint8_t item_len = 0;
    uint8_t counter = 0;
    uint8_t done = 0;
    char direction;

    // Keeps parsing tokens while one of the delimiters present in gps_item
    do
    {
        for (uint8_t i = 0; i < NMEA_MAX_LENGTH + 1; i++) {
            if (gps_item[i] == ',') {
                gps_item[i] = 0;
                item_len = i;
                break;
            }
            else if (gps_item[i] == 0) {
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
                data->time_ = (uint32_t)(atof(gps_item) * 100); // HHMMSS.SS format. Time is multiplied by 100.
                break;
            }

            case 2:
            {
                double latitude = (atof(gps_item)); // DDMM.MMMMMM
                data->latitude_.degrees_ = (int32_t)latitude / 100; // First 2 numbers are the latitude degrees
                data->latitude_.minutes_ = (int32_t)((latitude - data->latitude_.degrees_ * 100) * 100000); // Latitude minutes is multplied by 100000
                break;
            }

            case 3:    // Latitude direction
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
                data->longitude_.degrees_ = (int32_t)longitude / 100; // First 2 numbers are the longitude degrees
                data->longitude_.minutes_ = (int32_t)((longitude - data->longitude_.degrees_ * 100) * 100000); // Longitude minutes is multplied by 100000
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
                data->antennaAltitude_.altitude_ = (int32_t)(atof(gps_item) * 10); // Antenna altitude is multiplied by 10
                break;
            }

            case 10: // Antenna altitude unit
            {
                data->antennaAltitude_.unit_ = *gps_item;
                break;
            }

            case 11:
            {
                data->geoidAltitude_.altitude_ = (int32_t)(atof(gps_item) * 10); // Geoid altitude is multiplied by 10
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

    // Subtract geoid altitude from antenna altitude to get Height Above Ellipsoid (HAE)
    data->totalAltitude_.altitude_ = data->antennaAltitude_.altitude_ - data->geoidAltitude_.altitude_;
    data->totalAltitude_.unit_ = data->antennaAltitude_.unit_;

    memset(gpsTaskRxBuffer, 0, GPS_TASK_RX_BUFFER_SIZE);
    gpsDataMutex.Unlock();
}
