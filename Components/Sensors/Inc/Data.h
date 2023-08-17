#ifndef AVIONICS_INCLUDE_SOAR_DATA_H
#define AVIONICS_INCLUDE_SOAR_DATA_H

#include "SystemDefines.hpp"

/* Structs containing data primitives */

/*
 * IMPORTANT NOTE:
 *  Despite being typed as int32 or uint32 these are actually not integers.
 *  They represent fixed point decimal numbers.
 *
 * The specified precision is not consistent across all instruments,
 * please see the design manual for more information.
 */

typedef struct
{
    int32_t     accelX_;
    int32_t     accelY_;
    int32_t     accelZ_;
    int32_t     gyroX_;
    int32_t     gyroY_;
    int32_t     gyroZ_;
    int32_t     magnetoX_;
    int32_t     magnetoY_;
    int32_t     magnetoZ_;
    int32_t     time;
} AccelGyroMagnetismData;

typedef struct
{
    int32_t     pressure_;
    int32_t     temperature_;
    int32_t     time;
} BarometerData;

typedef struct
{
    int32_t     pressure_1;
} PressureTransducerData;

typedef struct
{
    int32_t     voltage_; // Volts * 1000, eg. 3300 == 3.3V
} BatteryData;

/* GPS Data */

#define NMEA_MAX_LENGTH 82

typedef struct
{
    int32_t    degrees_;
    int32_t    minutes_;
} LatLongType;

typedef struct
{
    int32_t     altitude_;
    char        unit_;
} AltitudeType;

typedef struct
{
    char            buffer_ [NMEA_MAX_LENGTH + 1];
    uint32_t        time_;
    LatLongType     latitude_;
    LatLongType     longitude_;
    AltitudeType    antennaAltitude_;
    AltitudeType    geoidAltitude_;
    AltitudeType    totalAltitude_;
} GpsData;


/* Data Containers */

/*
 * This is meant to act as a pointer to the other data structs.
 */

typedef struct
{
    AccelGyroMagnetismData*         accelGyroMagnetismData_;
    BarometerData*                  barometerData_;
    GpsData*                        gpsData_;
    PressureTransducerData* 		pressureTransducerData_;
    BatteryData*       				batteryData_;
} AllData;

typedef struct
{
    AccelGyroMagnetismData* accelGyroMagnetismData_;
    BarometerData*          barometerData_;
} ParachutesControlData;

#endif //AVIONICS_INCLUDE_SOAR_DATA_H
