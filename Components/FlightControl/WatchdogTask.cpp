/**
 ******************************************************************************
 * File Name          : WatchdogTask.cpp
 * Description        : Primary Watchdog task, default task for the system.
 ******************************************************************************
*/
#include "GPIO.hpp"
#include "SystemDefines.hpp"
#include "Timer.hpp"
#include "WatchdogTask.hpp"
#include "FlightTask.hpp"


/**
 * @brief Constructor for WatchdogTask
 */
WatchdogTask::WatchdogTask() : Task(WATCHDOG_TASK_QUEUE_DEPTH_OBJS)
{
}

/**
 * @brief Initialize the WatchdogTask
 */
void WatchdogTask::InitTask()
{
    // Make sure the task is not already initialized
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize watchdog task twice");

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
 * @brief This function is called if the heartbeat timer expires and it sends a command to ABORT the system
 * @param Automatically passes is timer handle for callback, should not be used
 */
void WatchdogTask::HeartbeatFailureCallback(TimerHandle_t rtTimerHandle)
{
    Timer::DefaultCallback(rtTimerHandle);
    // TODO : Remove this (open vetn and drain) later
    GPIO::Drain::Open();
    GPIO::Vent::Open();
    FlightTask::Inst().SendCommand(Command(CONTROL_ACTION, RSC_ANY_TO_ABORT));
}

/**
 * @brief Handles a command
 * @param cm Command reference to handle
 */
void WatchdogTask::HandleCommand(Command& cm)
{
    switch (cm.GetCommand()) {
    case HEARTBEAT_COMMAND: {
        HandleHeartbeat(cm.GetTaskCommand());
        break;
    }
    case TASK_SPECIFIC_COMMAND: {
        break;
    }
    case RADIOHB_CHANGE_PERIOD:
        SOAR_PRINT("HB Period Changed to %d s\n", (cm.GetTaskCommand()));
        heartbeatTimer->ChangePeriodMsAndStart((cm.GetTaskCommand()*1000));
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
        heartbeatTimer->ResetTimerAndStart();
        break;
    case RADIOHB_DISABLED:
        SOAR_PRINT("HEARTBEAT DISABLED \n");
        heartbeatTimer->Stop();
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
    uint32_t tempSecondCounter = 0; // TODO: Temporary counter, would normally be in HeartBeat task or HID Task, unless FlightTask is the HeartBeat task
    GPIO::LED1::Off();

    heartbeatTimer = new Timer(HeartbeatFailureCallback);
    heartbeatTimer->ChangePeriodMs(1200000);
    heartbeatTimer->Start();

    while (1) {
        //TODO: Move into HID Task
        GPIO::LED1::On();
        GPIO::LED2::On();
        GPIO::LED3::On();
        osDelay(500);
        GPIO::LED1::Off();
        GPIO::LED2::Off();
        GPIO::LED3::Off();
        osDelay(500);

        //Every cycle, print something out (for testing)
        SOAR_PRINT("FlightTask::Run() - [%d] Seconds\n", tempSecondCounter++);

        Command cm;

        //Wait forever for a command
        bool res = qEvtQueue->Receive(cm);

        if (res) {
            //Process the command
            HandleCommand(cm);
        }

    }
}
