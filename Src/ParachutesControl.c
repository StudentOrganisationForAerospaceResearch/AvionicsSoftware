#include <math.h>

#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "ParachutesControl.h"
#include "FlightPhase.h"
#include "Data.h"

// Pressure at spaceport america in 100*millibars on May 27, 2018
static const int SEA_LEVEL_PRESSURE = 101421.93903699999; //TODO: THIS NEEDS TO BE UPDATED AND RECORDED ON LAUNCH DAY
static const int MAIN_DEPLOYMENT_ALTITUDE = 457 + 1401; // Units in meters. Equivalent of 15000 ft + altitude of spaceport america.
static const int MONITOR_FOR_PARACHUTES_PERIOD = 200;
static const double KALMAN_GAIN[][2] =
{
    {0.105553059, 0.109271566},
    {0.0361533034, 0.0661198847},
    {0.000273178915, 0.618030079}
};

struct KalmanStateVector
{
    double altitude;
    double velocity;
    double acceleration;
};

int32_t readAccel(AccelGyroMagnetismData* data)
{
    if (osMutexWait(data->mutex_, 0) != osOK)
    {
        return -1;
    }

    int32_t accelX = data->accelX_;
    int32_t accelY = data->accelY_;
    int32_t accelZ = data->accelZ_;
    osMutexRelease(data->mutex_);

    int32_t accelMagnitude =
        sqrt(
            accelX * accelX +
            accelY * accelY +
            accelZ * accelZ
        );

    return accelMagnitude;
}

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



/**
 * Takes an old state vector and current state measurements and
 * converts them into a prediction of the rocket's current state.
 *
 * Params:
 *   oldState - (KalmanStateVector) Past altitude, velocity and acceleration
 *   currentAccel - (double) Measured acceleration
 *   currentAltitude - (double) Measured altitude
 *   dt - (double) Time since last step. In ms.
 *
 * Returns:
 *   newState - (KalmanStateVector) Current altitude, velocity and acceleration
 */
struct KalmanStateVector filterSensors(
    struct KalmanStateVector oldState,
    int32_t currentAccel,
    int32_t currentPressure,
    double dtMillis
)
{
    struct KalmanStateVector newState;

    double accelIn = (double) currentAccel / 1000 * 9.8; // Milli-g -> g -> m/s

    // Convert from 100*millibars to m. This may or may not be right, depending on where you look. Needs testing
    double altIn = (double) 44307.69396 * (1 - pow(currentPressure / SEA_LEVEL_PRESSURE, 0.190284));

    // Convert from ms to s
    double dt = dtMillis / 1000;


    // Propagate old state using simple kinematics equations
    newState.altitude = oldState.altitude + oldState.velocity * dt + 0.5 * dt * dt * oldState.acceleration;
    newState.velocity = oldState.velocity + oldState.acceleration * dt;
    newState.acceleration = oldState.acceleration;

    // Calculate the difference between the new state and the measurements
    double baroDifference = altIn - newState.altitude;
    double accelDifference = accelIn - newState.acceleration;

    // Minimize the chi2 error by means of the Kalman gain matrix
    newState.altitude = newState.altitude + KALMAN_GAIN[0][0] * baroDifference + KALMAN_GAIN[0][1] * accelDifference;
    newState.velocity = newState.velocity + KALMAN_GAIN[1][0] * baroDifference + KALMAN_GAIN[1][1] * accelDifference;
    newState.acceleration = newState.velocity + KALMAN_GAIN[2][0] * baroDifference + KALMAN_GAIN[2][1] * accelDifference;

    return newState;
}

/**
 * Takes the current state vector and determines if apogee has been reached.
 *
 * Params:
 *   state - (KalmanStateVector) Past altitude, velocity and acceleration
 *
 * Returns:
 *   - (int32_t) 1 if apogee has been reached, 0 if not.
 */
int32_t detectApogee(struct KalmanStateVector state)
{
    // Monitor for when to deploy drogue chute. Simple velocity tolerance, looking for a minimum.
    if (state.velocity < 25)
    {
        return 1;
    }

    return 0;
}

/**
 * Takes the current state vector and determines if main chute should be released.
 *
 * ***NOTE: This is determined by the constant MAIN_DEPLOYMENT_ALTITUDE,
 *           which should be verified before launch.
 *
 * Params:
 *   state - (KalmanStateVector) Past altitude, velocity and acceleration
 *
 * Returns:
 *   - (int32_t) 1 if main chute should be deployed, 0 if not.
 */
int32_t detectMainDeploymentAltitude(struct KalmanStateVector state)
{
    // Monitor for when to deploy main chute. Simply look for less than desired altitude.
    if (state.altitude < MAIN_DEPLOYMENT_ALTITUDE)
    {
        return 1;
    }

    return 0;
}

void ejectDrogueParachute()
{
    HAL_GPIO_WritePin(DROGUE_PARACHUTE_GPIO_Port, DROGUE_PARACHUTE_Pin, GPIO_PIN_SET);  // high signal causes high current to ignite e-match
}

void ejectMainParachute()
{
    HAL_GPIO_WritePin(MAIN_PARACHUTE_GPIO_Port, MAIN_PARACHUTE_Pin, GPIO_PIN_SET);  // high signal causes high current to ignite e-match
}

/**
 * This routine just waits for the current flight phase to get out of PRELAUNCH
 */
void parachutesControlPrelaunchRoutine()
{
    uint32_t prevWakeTime = osKernelSysTick();

    for (;;)
    {
        osDelayUntil(&prevWakeTime, MONITOR_FOR_PARACHUTES_PERIOD);

        if (getCurrentFlightPhase() != PRELAUNCH)
        {
            // Ascent has begun
            return;
        }
    }
}

void parachutesControlBurnRoutine(
    AccelGyroMagnetismData* accelGyroMagnetismData,
    BarometerData* barometerData,
    struct KalmanStateVector state
)
{
    uint32_t prevWakeTime = osKernelSysTick();

    for (;;)
    {
        osDelayUntil(&prevWakeTime, MONITOR_FOR_PARACHUTES_PERIOD);

        if (getCurrentFlightPhase() != BURN)
        {
            return;
        }

        int32_t currentAccel = readAccel(accelGyroMagnetismData);
        int32_t currentPressure = readPressure(barometerData);

        if (currentAccel == -1 || currentPressure == -1)
        {
            // failed to read values
            continue;
        }

        filterSensors(state, currentAccel, currentPressure, MONITOR_FOR_PARACHUTES_PERIOD);
    }
}


/**
 * This routine monitors for apogee.
 * Once apogee has been detected,
 * eject the drogue parachute and update the current flight phase.
 */
void parachutesControlCoastRoutine(
    AccelGyroMagnetismData* accelGyroMagnetismData,
    BarometerData* barometerData,
    struct KalmanStateVector state
)
{
    uint32_t prevWakeTime = osKernelSysTick();

    for (;;)
    {
        osDelayUntil(&prevWakeTime, MONITOR_FOR_PARACHUTES_PERIOD);

        int32_t currentAccel = readAccel(accelGyroMagnetismData);
        int32_t currentPressure = readPressure(barometerData);

        if (currentAccel == -1 || currentPressure == -1)
        {
            // failed to read values
            continue;
        }

        filterSensors(state, currentAccel, currentPressure, MONITOR_FOR_PARACHUTES_PERIOD);

        if (detectApogee(state))
        {
            ejectDrogueParachute();
            newFlightPhase(DROGUE_DESCENT);
            return;
        }
    }
}

/**
 * This routine detects reaching a certain altitude
 * Once that altitude has been reached, eject the main parachute
 * and update the current flight phase.
 */
void parachutesControlDrogueDescentRoutine(
    AccelGyroMagnetismData* accelGyroMagnetismData,
    BarometerData* barometerData,
    struct KalmanStateVector state
)
{
    uint32_t prevWakeTime = osKernelSysTick();

    for (;;)
    {
        osDelayUntil(&prevWakeTime, MONITOR_FOR_PARACHUTES_PERIOD);


        int32_t currentAccel = readAccel(accelGyroMagnetismData);
        int32_t currentPressure = readPressure(barometerData);

        if (currentAccel == -1 || currentPressure == -1)
        {
            // failed to read values
            continue;
        }

        filterSensors(state, currentAccel, currentPressure, MONITOR_FOR_PARACHUTES_PERIOD);

        // detect 4600 ft above sea level and eject main parachute
        if (detectMainDeploymentAltitude(state))
        {
            ejectMainParachute();
            newFlightPhase(MAIN_DESCENT);
            return;
        }
    }
}

void parachutesControlTask(void const* arg)
{
    ParachutesControlData* data = (ParachutesControlData*) arg;
    struct KalmanStateVector state;

    for (;;)
    {
        switch (getCurrentFlightPhase())
        {
            case PRELAUNCH:
                parachutesControlPrelaunchRoutine();
                break;

            case BURN:
                parachutesControlBurnRoutine(
                    data->accelGyroMagnetismData_,
                    data->barometerData_,
                    state
                );
                break;

            case COAST:
                parachutesControlCoastRoutine(
                    data->accelGyroMagnetismData_,
                    data->barometerData_,
                    state
                );
                break;

            case DROGUE_DESCENT:
                parachutesControlDrogueDescentRoutine(
                    data->accelGyroMagnetismData_,
                    data->barometerData_,
                    state
                );

                break;

            case MAIN_DESCENT:
            case ABORT:
                // do nothing
                osDelay(MONITOR_FOR_PARACHUTES_PERIOD);
                break;

            default:
                break;
        }
    }
}
