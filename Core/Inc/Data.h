#pragma once

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
    osMutexId   mutex_;
    int32_t     accelX_;
    int32_t     accelY_;
    int32_t     accelZ_;
    int32_t     gyroX_;
    int32_t     gyroY_;
    int32_t     gyroZ_;
    int32_t     magnetoX_;
    int32_t     magnetoY_;
    int32_t     magnetoZ_;
} AccelGyroMagnetismData;

typedef struct
{
    osMutexId   mutex_;
    int32_t     pressure_;
    int32_t     temperature_;
} BarometerData;

typedef struct
{
    osMutexId   mutex_;
    int32_t     voltage_;
} CombustionChamberPressureData;

typedef struct
{
    osMutexId   mutex_;
    int32_t     pressure_;
} BatteryVoltageData;

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
    osMutexId       mutex_;
    char            buffer_ [NMEA_MAX_LENGTH + 1];
    uint32_t        time_;
    LatLongType     latitude_;
    LatLongType     longitude_;
    AltitudeType    antennaAltitude_;
    AltitudeType    geoidAltitude_;
    AltitudeType    totalAltitude_;
    uint8_t         parseFlag_;
} GpsData;

typedef struct
{
    osMutexId   mutex_;
    int32_t     pressure_;
} OxidizerTankPressureData;

/* Data Containers */

/*
 * This is meant to act as a pointer to the other data structs.
 */

typedef struct
{
    AccelGyroMagnetismData*         accelGyroMagnetismData_;
    BarometerData*                  barometerData_;
    CombustionChamberPressureData*  combustionChamberPressureData_;
    GpsData*                        gpsData_;
    OxidizerTankPressureData*       oxidizerTankPressureData_;
} AllData;

typedef struct
{
    AccelGyroMagnetismData* accelGyroMagnetismData_;
    BarometerData*          barometerData_;
} ParachutesControlData;


/* Structs -------------------------------------------------------------------*/
typedef struct{
    int32_t accelX;
    int32_t accelY;
    int32_t accelZ;
    int32_t gyroX;
    int32_t gyroY;
    int32_t gyroZ;
    int32_t magnetoX;
    int32_t magnetoY;
    int32_t magnetoZ;
    int32_t barometerPressure;
    int32_t barometerTemperature;
    int32_t combustionChamberPressure;
    int32_t oxidizerTankPressure;
    int32_t gps_time;
    int32_t latitude_degrees;
    int32_t latitude_minutes;
    int32_t longitude_degrees;
    int32_t longitude_minutes;
    int32_t antennaAltitude;
    int32_t geoidAltitude;
    int32_t altitude;
    uint8_t currentFlightPhase;
    int32_t tick;

} LogEntry; // LogEntry holds data from AllData that is to be logged
