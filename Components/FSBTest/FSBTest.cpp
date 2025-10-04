/**
 ********************************************************************************
 * @file    FSBProtocolTask.cpp
 * @author  jaddina
 * @date    Sep 13, 2025
 * @brief
 ********************************************************************************
 */

/************************************
 * INCLUDES
 ************************************/
#include "FSBTest.hpp"
#include "SystemDefines.hpp"
#include "Command.hpp"

#include "DataBroker.hpp"
#include "Task.hpp"

/************************************
 * PRIVATE MACROS AND DEFINES
 ************************************/

/************************************
 * VARIABLES
 ************************************/

/************************************
 * FUNCTION DECLARATIONS
 ************************************/



/************************************
 * FUNCTION DEFINITIONS
 ************************************/
FSBProtocolTask::FSBProtocolTask():Task(TASK_FSB_PROTOCOL_DEPTH_OBJS)
{

}

/**
 * @brief Initialize the FSBProtocolTask
 *        Do not modify this function aside from adding the task name
 */
void FSBProtocolTask::InitTask()
{
    // Make sure the task is not already initialized
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize watchdog task twice");

    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)FSBProtocolTask::RunTask,
            (const char*)"FSBProtocolTask",
            (uint16_t)TASK_FSB_PROTOCOL_DEPTH_WORDS,
            (void*)this,
            (UBaseType_t)TASK_FSB_PROTOCOL_PRIORITY,
            (TaskHandle_t*)&rtTaskHandle);

                SOAR_ASSERT(rtValue == pdPASS, "FSBProtocolTask::InitTask() - xTaskCreate() failed");
}

void FSBProtocolTask::Run(void * pvParams){

    while (1) {
        /* Process commands in blocking mode */
        Command cm;
        bool res = qEvtQueue->ReceiveWait(cm);
        if(res){
        	SOAR_PRINT("here");
        	HandleCommand(cm);
        }
    }
}

void FSBProtocolTask::HandleCommand(Command& cm){


	switch(cm.GetTaskCommand()){

		case PUBLISH_PRESSURE:

			PressureData pd;
			pd.pressure = 140;
			SOAR_PRINT("Pressure published");

			DataBroker::Publish<PressureData>(&pd);
			break;

		case PUBLISH_IMU:

			IMUData imu;
			imu.accelX = 100;
			imu.accelY = 150;
			imu.accelZ = 200;
			SOAR_PRINT("IMU published");
			DataBroker::Publish<IMUData>(&imu);
			break;


		case PUBLISH_THERMOCOUPLE:

			ThermocoupleData tc;
			tc.temperature = 150;
			SOAR_PRINT("Temperature published");

			DataBroker::Publish<ThermocoupleData>(&tc);
			break;
	}

}


