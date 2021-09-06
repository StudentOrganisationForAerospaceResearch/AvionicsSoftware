#pragma once

/* Structs containing data primitives */

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
    int32_t     pressure_;
} CombustionChamberPressureData;

typedef struct
{
    osMutexId   mutex_;
    uint32_t     voltage_;
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
