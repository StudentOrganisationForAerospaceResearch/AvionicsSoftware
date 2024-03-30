/**
  ******************************************************************************
  * File Name          : KalmanFilter.cpp
  *
  *Author			   : Andrey Dimanchev
  * Description        : Filters and fuses sensor data based on a Kalman Filter
  ******************************************************************************
*/
/* Includes ------------------------------------------------------------------*/
#include "KalmanFilterTask.hpp"

#include "main.h"
#include "Data.h"
#include "Task.hpp"
#include <string.h>


KalmanFilterTask::KalmanFilterTask(){
	data = (AccelGyroMagnetismData*)soar_malloc(sizeof(AccelGyroMagnetismData));
}

/**
 * @brief Creates a task for the FreeRTOS Scheduler
 */
void KalmanFilterTask::InitTask() : Task(TASK_KALMAN_FILTER_QUEUE_DEPTH_OBJS)
{
    // Make sure the task is not already initialized
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize Kalman Filter task twice");

    // Start the task
    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)KalmanFilterTask::RunTask,
            (const char*)"KalmanFilterTask",
            (uint16_t)TASK_KALMAN_FILTER_STACK_DEPTH_WORDS,
            (void*)this,
            (UBaseType_t)TASK_KALMAN_FILTER_PRIORITY,
            (TaskHandle_t*)&rtTaskHandle);

    //Ensure creation succeded
    SOAR_ASSERT(rtValue == pdPASS, "KalmanFilterTask::InitTask() - xTaskCreate() failed");
}

/**
 * @brief KalmanFilterTask run loop
 * @param pvParams Currently unused task context
 */
void KalmanFilterTask::Run(void* pvParams)
{
    //Delay before Kalman Filter init
    osDelay(100);

    //Task run loop
    while (1) {
        Command cm;

        //Wait forever for a command
        qEvtQueue->ReceiveWait(cm);

        //Process the command
        HandleCommand(cm);
    }
}

/**
 * @brief Handles a command
 * @param cm Command reference to handle
 */
void KalmanFilterTask::HandleCommand(Command& cm)
{
    //TODO: Since this task will stall for a few milliseconds, we may need a way to eat the whole queue (combine similar eg. REQUEST commands and eat to WDG command etc)
    //TODO: Maybe a HandleEvtQueue instead that takes in the whole queue and eats the whole thing in order of non-blocking to blocking

    //Switch for the GLOBAL_COMMAND
    switch (cm.GetCommand()) {
    case TASK_SPECIFIC_COMMAND: {
    	HandleRequestCommand(cm.GetTaskCommand());
        break;
    }
    default:
        SOAR_PRINT("KalmanFilterTask - Received Unsupported Command {%d}\n", cm.GetCommand());
        break;
    }

    //No matter what happens, we must reset allocated data
    cm.Reset();
}

/**
 * @brief Handles a Request Command
 * @param taskCommand The command to handle
 */
void KalmanFilterTask::HandleRequestCommand(uint16_t taskCommand)
{
    //Switch for task specific command within DATA_COMMAND
    switch (taskCommand) {
    case READ_NEW_SAMPLE:
        ReadSensors();
        break;

    default:
        SOAR_PRINT("IMUTask - Received Unsupported REQUEST_COMMAND {%d}\n", taskCommand);
        break;
    }
}

void KalmanFilterTask::ReadSensors(){
	SOAR_PRINT("\t-- IMU Data --\n");
	SOAR_PRINT(" Accel (x,y,z) : (%d, %d, %d) milli-Gs\n", data->accelX_, data->accelY_, data->accelZ_);
	SOAR_PRINT(" Gyro (x,y,z)  : (%d, %d, %d) milli-deg/s\n", data->gyroX_, data->gyroY_, data->gyroZ_);
	SOAR_PRINT(" Mag (x,y,z)   : (%d, %d, %d) milli-gauss\n", data->magnetoX_, data->magnetoY_, data->magnetoZ_);

}
