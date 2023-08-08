/**
 ******************************************************************************
 * File Name          : GPSTask.hpp
 * Description        :
 ******************************************************************************
*/
#ifndef SOAR_SENSOR_GPS_TASK_HPP_
#define SOAR_SENSOR_GPS_TASK_HPP_

/* Includes -----------------------------------------------------------------*/
#include "Data.h"
#include "Task.hpp"
#include "SystemDefines.hpp"

/* GPS Data Flash Log Format -----------------------------------------------------------------*/
typedef struct
{
    uint32_t        time_;
    LatLongType     latitude_;
    LatLongType     longitude_;
    AltitudeType    antennaAltitude_;
    AltitudeType    geoidAltitude_;
    AltitudeType    totalAltitude_;
} GPSDataFlashLog;

/* Configuration ------------------------------------------------------------*/
constexpr uint16_t READ_GPS_PERIOD_MS = 250;    // Period to read GPS data

/* Macros / Enumerations ----------------------------------------------------*/
constexpr uint16_t NMEA_MAX_LENGTH_BYTES = 82;
constexpr uint16_t GPS_TASK_RX_BUFFER_SIZE = NMEA_MAX_LENGTH_BYTES + 1;

// External Request Commands
enum GPS_REQUEST_COMMANDS {
    GPS_NONE = 0,
    GPS_REQUEST_SAMPLE_INVALID,// Get a new GPS sample (does nothing for GPS)
    GPS_REQUEST_TRANSMIT,    // Send the current GPS data over the Radio
    GPS_REQUEST_DEBUG,        // Send the current GPS data over the Debug UART
    GPS_REQUEST_FLASH_LOG,
};

// Internal Events
enum GPS_DATA_COMMANDS {
    GPS_TASK_COMMAND_NONE = 0,
    EVENT_GPS_RX_PARSE_READY  // Notification that NMEA data is ready to be parsed
};

/**
 * @brief GPS Task
 */
class GPSTask : public Task
{
public:
    static GPSTask& Inst() {
        static GPSTask inst;
        return inst;
    }

    void InitTask();

    //Functions exposed to HAL callbacks
    void HandleGPSRxComplete();

protected:
    static void RunTask(void* pvParams) { GPSTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();

    void Run(void* pvParams);    // Main run code

    // Command Handling
    void HandleCommand(Command& cm);
    void HandleRequestCommand(uint16_t taskCommand);

    void TransmitProtocolData();
    void LogDataToFlash();

    // Data Transfer
    bool ReceiveData();

    // GPS Data Handling
    void ParseGpsData();

    // Member variables
    Mutex gpsDataMutex;
    uint8_t gpsTaskRxBuffer[GPS_TASK_RX_BUFFER_SIZE];
    GpsData* data;

private:
    // Private Functions
    GPSTask();        // Private constructor
    GPSTask(const GPSTask&);                        // Prevent copy-construction
    GPSTask& operator=(const GPSTask&);            // Prevent assignment

    // Private Variables

};

#endif // SOAR_SENSOR_GPS_TASK_HPP_
