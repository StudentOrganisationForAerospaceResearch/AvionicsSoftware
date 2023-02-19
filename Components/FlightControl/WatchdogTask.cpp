/**
 ******************************************************************************
 * File Name          : WatchdogTask.cpp
 * Description        : Primary flight task, default task for the system.
 ******************************************************************************
*/
#include "GPIO.hpp"
#include "SystemDefines.hpp"
#include "Timer.hpp"
#include "WatchdogTask.hpp"
#include "FlightTask.hpp"


/**
 * @brief Initialize the WatchdogTask
 * @params Must pass in the timer period, If no heartbeat in received within this period then the timerr will be reset
 */
WatchdogTask::WatchdogTask(){}

/**
 * @brief Initialize the FlightTask
 */
void WatchdogTask::InitTask()
{
    // Make sure the task is not already initialized
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize flight task twice");

    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)WatchdogTask::RunTask,
            (const char*)"WatchdogTask",
            (uint16_t)WATCHDOG_TASK_STACK_DEPTH_WORDS,
            (void*)this,
            (UBaseType_t)WATCHDOG_TASK_RTOS_PRIORITY,
            (TaskHandle_t*)&rtTaskHandle);

    SOAR_ASSERT(rtValue == pdPASS, "WatchdogTask::InitTask() - xTaskCreate() failed");
}

/**
 * @brief This function is called if the heartbeat timer expires and it sends a command to reset the system
 * @param Automatically passes is timer handle for callback, should not be used
 */
void WatchdogTask::HeartbeatFailureCallback(TimerHandle_t rtTimerHandle)
{

    SOAR_PRINT("The system lost its heartbeat and had to reset!!!\n");
//    SOAR_ASSERT(false ,"The system lost its heartbeat and had to reset!!! \n");
    Timer::DefaultCallback(rtTimerHandle);
    WatchdogTask::Inst().SendCommand(Command(CONTROL_ACTION, RSC_ANY_TO_ABORT));
}

/**
 * @brief Handles a command
 * @param cm Command reference to handle
 */
void WatchdogTask::HandleCommand(Command& cm)
{
    switch (cm.GetCommand()) {
    case REQUEST_COMMAND: {
        HandleHeartbeat(cm.GetTaskCommand());
    }
    case TASK_SPECIFIC_COMMAND: {
        break;
    }
    case RADIOHB_CHANGE_PERIOD:
        SOAR_PRINT("HB Period Changed \n");
        heartbeatTimer.ChangePeriodMsAndStart((cm.GetTaskCommand()*1000));
        break;
    default:
        SOAR_PRINT("WatchdogTask - Received Unsupported Command {%d}\n", cm.GetCommand());
        break;
    }

    //No matter what we happens, we must reset allocated data
    cm.Reset();
}

void WatchdogTask::HandleHeartbeat(uint16_t taskCommand)
{
    switch (taskCommand) {
    case RADIOHB_REQUEST:
        SOAR_PRINT("HEARTBEAT RECEIVED \n");
        heartbeatTimer.ResetTimerAndStart();
        break;
    case RADIOHB_DISABLED:
        SOAR_PRINT("HEARTBEAT DISABLED \n");
        heartbeatTimer.Stop();
        break;
    default:
        SOAR_PRINT("WatchdogTask - Received Unsupported REQUEST_COMMAND {%d}\n", taskCommand);
        break;
    }
}

/**
 * @brief Instance Run loop for the Watchdog Task, runs on scheduler start as long as the task is initialized.
 * @param pvParams RTOS Passed void parameters, contains a pointer to the object instance, should not be used
 */
void WatchdogTask::Run(void * pvParams)
{
    GPIO::LED1::Off();

    heartbeatTimer = Timer(HeartbeatFailureCallback);
    heartbeatTimer.ChangePeriodMs(5000);
    heartbeatTimer.Start();


    while (1) {
        //Every cycle, print something out (for testing)
//        SOAR_PRINT("WatchdogTask::Run() - [%d] Seconds\n", tempSecondCounter++);

        Command cm;

        //Wait forever for a command
        qEvtQueue->ReceiveWait(cm);

        //Process the command
        HandleCommand(cm);


        // TODO: Message beeps
    }
}
