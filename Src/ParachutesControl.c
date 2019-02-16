/**
  ******************************************************************************
  * File Name          : ReadBarometer.c
  * Description        : This file contains constants and functions designed to
  *                      obtain accurate pressure and temperature readings from
  *                      the MS5607-02BA03 barometer on the flight board. A
  *                      thread task is included that will constantly loop,
  *                      reading and updating the passed BarometerData struct.
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/

#include <math.h>

#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "ParachutesControl.h"
#include "FlightPhase.h"
#include "Data.h"
#include "AltitudeKalmanFilter.h"

/* Macros --------------------------------------------------------------------*/

/* Constants -----------------------------------------------------------------*/

// Pressure at spaceport america in 100*millibars on May 27, 2018
static const int MAIN_DEPLOYMENT_ALTITUDE = 457 + 1401; // Units in meters. Equivalent of 15000 ft + altitude of spaceport america.
static const int MONITOR_FOR_PARACHUTES_PERIOD = 200;

/* Variables -----------------------------------------------------------------*/

/* Structs -------------------------------------------------------------------*/

/* Prototypes ----------------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/

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

/**
 * Releases the drouge parachute on the rocket.
 */
void ejectDrogueParachute()
{
    HAL_GPIO_WritePin(DROGUE_PARACHUTE_GPIO_Port, DROGUE_PARACHUTE_Pin, GPIO_PIN_SET);  // high signal causes high current to ignite e-match
}

/**
 * Releases the main parachute on the rocket.
 */
void ejectMainParachute()
{
    HAL_GPIO_WritePin(MAIN_PARACHUTE_GPIO_Port, MAIN_PARACHUTE_Pin, GPIO_PIN_SET);  // high signal causes high current to ignite e-match
}

/**
 * Waits for the current flight phase to get out of PRELAUNCH
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

/**
 * Desc
 *
 * Params:
 *   accelGyromMagnetismData	- (AccelGyroMagnetismData*) Pointer to the IMU data struct to read the acceleration from.
 *   barometerData				- (barometerData*) Pointer to the barometer data struct to read the pressure from.
 *   state						- (KalmanStateVector) The current acceleration, pressure, and altitude state of the rocket.
 */
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
 * Monitors for apogee. Once apogee has been detected,
 * eject the drogue parachute and update the current flight phase.
 *
 * Params:
 *   accelGyromMagnetismData	- (AccelGyroMagnetismData*) Pointer to the IMU data struct to read the acceleration from.
 *   barometerData				- (barometerData*) Pointer to the barometer data struct to read the pressure from.
 *   state						- (KalmanStateVector) The current acceleration, pressure, and altitude state of the rocket.
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
 * This routine detects reaching a certain altitude.
 * Once that altitude has been reached, eject the main parachute
 * and update the current flight phase.
 *
 * Params:
 *   accelGyromMagnetismData	- (AccelGyroMagnetismData*) Pointer to the IMU data struct to read the acceleration from.
 *   barometerData				- (barometerData*) Pointer to the barometer data struct to read the pressure from.
 *   state						- (KalmanStateVector) The current acceleration, pressure, and altitude state of the rocket.
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

/**
 * This function is to be used as a thread task that reads and updates
 * a kalman state vector, which it uses to detect the apogee of the rocket's
 * flight path. Once apogee is detected, a drouge parachute will be released.
 * Some time after this, the main parachute will also be deployed.
 *
 * Params:
 *   arg - (void const*) A pointer to the ParachutesControlData struct that will be used.
 */
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
