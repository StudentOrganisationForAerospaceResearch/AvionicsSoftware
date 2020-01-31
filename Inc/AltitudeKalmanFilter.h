#pragma once
#include "Data.h"

struct KalmanStateVector
{
    double altitude;
    double velocity;
    double acceleration;
};

int32_t readAccel(AccelGyroMagnetismData* data);
int32_t readPressure(BarometerData* data);
struct KalmanStateVector filterSensors(
    struct KalmanStateVector oldState,
    int32_t currentAccel,
    int32_t currentPressure,
    double dtMillis
);
