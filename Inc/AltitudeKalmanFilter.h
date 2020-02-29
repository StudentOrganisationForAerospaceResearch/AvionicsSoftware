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
void filterSensors(
    struct KalmanStateVector* oldState,
    double currentAccel,
    double currentPressure,
    double dtMillis
);
