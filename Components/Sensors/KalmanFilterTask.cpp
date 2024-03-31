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

#include "Unscented.h"



#include <chrono>
using namespace std::chrono;


KalmanFilterTask::KalmanFilterTask() : Task(TASK_KALMAN_FILTER_QUEUE_DEPTH_OBJS)
{
	data = (AccelGyroMagnetismData*)soar_malloc(sizeof(AccelGyroMagnetismData));
}

/**
 * @brief Creates a task for the FreeRTOS Scheduler
 */
void KalmanFilterTask::InitTask()
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
    	osDelay(100);

    	auto start = high_resolution_clock::now();

    	Test();

    	auto stop = high_resolution_clock::now();

    	auto duration = duration_cast<microseconds>(stop - start);

    	// To get the value of duration use the count()
    	// member function on the duration object
//    	cout << duration.count() << endl;
//        Command cm;
//
//        //Wait forever for a command
//        qEvtQueue->ReceiveWait(cm);
//
//        //Process the command
//        HandleCommand(cm);
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
	SOAR_PRINT(" here");

}

void KalmanFilterTask::Test(){
    VectorXf X0u(6);
    X0u << 400, 0, 0, -300, 0, 0;

    MatrixXf P0u(6, 6);
    P0u << 500, 0, 0, 0, 0, 0,
        0, 500, 0, 0, 0, 0,
        0, 0, 500, 0, 0, 0,
        0, 0, 0, 500, 0, 0,
        0, 0, 0, 0, 500, 0,
        0, 0, 0, 0, 0, 500;

    VectorXf Zu(2);

    MatrixXf Ru(2, 2);
    Ru << 25, 0,
        0, 0.00007569;

    MatrixXf Fu(6, 6);
    Fu << 1, 1, 0.5, 0, 0, 0,
        0, 1, 1, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 1, 1, 0.5,
        0, 0, 0, 0, 1, 1,
        0, 0, 0, 0, 0, 1;

    MatrixXf Qau(6, 6);
    Qau << 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0.04, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0.04;

    double posu[2][35] = {{502.55, 477.34, 457.21, 442.94, 427.27, 406.05, 400.73, 377.32, 360.27, 345.93, 333.34, 328.07, 315.48,
                          301.41, 302.87, 304.25, 294.46, 294.29, 299.38, 299.37, 300.68, 304.1, 301.96, 300.3, 301.9, 296.7, 297.07,
                          295.29, 296.31, 300.62, 292.3, 298.11, 298.07, 298.92, 298.04},
                         {-0.9316, -0.8977, -0.8512, -0.8114, -0.7853, -0.7392, -0.7052, -0.6478, -0.59, -0.5183, -0.4698, -0.3952, -0.3026,
                          -0.2445, -0.1626, -0.0937, 0.0085, 0.0856, 0.1675, 0.2467, 0.329, 0.4149, 0.504, 0.5934, 0.667, 0.7537, 0.8354,
                          0.9195, 1.0039, 1.0923, 1.1546, 1.2564, 1.3274, 1.409, 1.5011}};

    KalmanU car(6, 2);
    car.init(X0u, P0u, Zu, Ru, Fu, Qau);
    for (int k = 0; k < 35; k++)
    {
        Zu << posu[0][k], posu[1][k];
        car.update();
    }

//    SOAR_PRINT(car.getState());
//    cout << car.getState() << endl;
//    cout << car.getCov() << endl;
}
