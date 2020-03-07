#pragma once
#include "Data.h"
#include "Vec3d.h"

struct KalmanStateVector
{
    double altitude;
    struct Vec3d velocity;
    struct Vec3d acceleration;
};

void zeroState(struct KalmanStateVector *state);

struct Vec3d readAccel(AccelGyroMagnetismData* data);
int32_t readPressure(BarometerData* data);
void filterSensors(
    struct KalmanStateVector* oldState,
    struct Vec3d currentAccel,
    double currentPressure,
    double dtMillis
);
