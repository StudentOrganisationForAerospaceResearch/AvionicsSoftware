/**
  ******************************************************************************
  * File Name          : AltitudeKalmanFilter.c
  * Description        : This file contains an implementation of a Kalman filter
  *                      designed to obtain accurate altitude readings from both
  *                      the accelerometer and barometer on board the rocket.
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/

#include <math.h>

#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "AltitudeKalmanFilter.h"
#include "Data.h"
#include "Vec3d.h"

/* Macros --------------------------------------------------------------------*/

/* Constants -----------------------------------------------------------------*/



static const double KALMAN_GAIN[][2] =
{
    {0.105553059, 0.109271566},
    {0.0361533034, 0.0661198847},
    {0.000273178915, 0.618030079}
};

/* Variables -----------------------------------------------------------------*/

/* Structs -------------------------------------------------------------------*/

/* Prototypes ----------------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/

/**
 * Reads the acceleration from the passed IMU data struct, and provides the magnitude.
 *
 * Params:
 *   data 			- (AccelGyroMagnetismData*) Pointer to the IMU struct to be read from.
 *
 * Returns:
 *   accelMagnitude	- (int32_t) The magnitude of the read acceleration.
 */
//int32_t readAccel(AccelGyroMagnetismData* data)
struct Vec3d readAccel(AccelGyroMagnetismData* data)
{
    struct Vec3d accel;
    if (osMutexWait(data->mutex_, 0) != osOK)
    {
        accel.vb = -1;
        return accel;
    }

    accel.x = data->accelX_;
    accel.y = data->accelY_;
    accel.z = data->accelZ_;
    osMutexRelease(data->mutex_);

    accel.vb = 1;
    return accel;

    // TODO Take out after everything is finalized
//    int32_t accelX = data->accelX_;
//    int32_t accelY = data->accelY_;
//    int32_t accelZ = data->accelZ_;
//    osMutexRelease(data->mutex_);

//    int32_t accelMagnitude =
//        sqrt(
//            accelX * accelX +
//            accelY * accelY +
//            accelZ * accelZ
//        );

//    return accelMagnitude;
}

/**
 * Reads the pressure from the passed barometer data struct.
 *
 * Params:
 *   data 		- (BarometerData*) Pointer to the barometer struct to be read from.
 *
 * Returns:
 *   pressure 	- (int32_t) The read pressure.
 */
int32_t readPressure(BarometerData* data)
{
    if (osMutexWait(data->mutex_, 0) != osOK)
    {
        return -1;
    }

    int32_t pressure = data->pressure_;
    osMutexRelease(data->mutex_);

    return (int32_t)pressure;
}

void zeroState(struct KalmanStateVector *state)
{
    state->altitude = 0;
    state->velocity = vectorZero();
    state->acceleration = vectorZero();
}

/**
 * Takes an old state vector and current state measurements and
 * converts them into a prediction of the rocket's current state.
 *
 * TODO change params
 * Params:
 *   oldState 			- (KalmanStateVector) Past altitude, velocity and acceleration
 *   currentAccel 		- (double) Measured acceleration
 *   currentAltitude 	- (double) Measured altitude
 *   dt 				- (double) Time since last step. In ms.
 *
 * Returns:
 *   newState 			- (KalmanStateVector) Current altitude, velocity and acceleration
 */
void filterSensors(
    struct KalmanStateVector* state,
    struct Vec3d currentAccel,
    double currentAlt,
    double dtMillis
)
{
    struct KalmanStateVector newState;

    // Convert from ms to s
    double dt = dtMillis / 1000;

//    newState.altitude = altIn;
//    newState.acceleration = accelIn;


    // Propagate old state using simple kinematics equations
//    newState.altitude = oldState.altitude + oldState.velocity * dt + 0.5 * dt * dt * oldState.acceleration;
//    newState.velocity = oldState.velocity + oldState.acceleration * dt;
//    newState.acceleration = oldState->acceleration;

    // Calculate the difference between the new measurements and the old state
    double baroDifference =  currentAlt - state->altitude;
    struct Vec3d accelDifference = vectorSub(currentAccel, state->acceleration);

    // Minimize the chi2 error by means of the Kalman gain matrix
//    newState.altitude = newState.altitude + KALMAN_GAIN[0][0] * baroDifference + KALMAN_GAIN[0][1] * accelDifference;
//    newState.velocity = newState.velocity + KALMAN_GAIN[1][0] * baroDifference + KALMAN_GAIN[1][1] * accelDifference;
//    newState.acceleration = newState.velocity + KALMAN_GAIN[2][0] * baroDifference + KALMAN_GAIN[2][1] * accelDifference;

    // TODO condense back down into one liner
    double tmpBaroDiff = KALMAN_GAIN[0][0] * baroDifference;
    struct Vec3d tmpAccelDiff = vectorMultScalar(KALMAN_GAIN[0][1], accelDifference);
    newState.altitude = state->altitude + tmpBaroDiff + vectorMagnitude(tmpAccelDiff);

    // TODO implement velocity when we finish its calculations for now just return old state
    newState.velocity = state->velocity;
    //newState.velocity = newState.velocity + KALMAN_GAIN[1][0] * baroDifference + KALMAN_GAIN[1][1] * accelDifference;

    // TODO condense back down into one liner
    tmpBaroDiff = KALMAN_GAIN[2][0] * baroDifference;
    tmpAccelDiff = vectorMultScalar(KALMAN_GAIN[2][1], accelDifference);
    newState.acceleration = vectorUniformScale(tmpBaroDiff, vectorAdd(state->acceleration, tmpAccelDiff));

    *state = newState;
}

