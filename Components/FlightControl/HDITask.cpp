/**
******************************************************************************
* File Name          : HDITask.cpp
* Description        : Primary flight task, default task for the system.
******************************************************************************
*/
#include "HDITask.hpp"
#include "GPIO.hpp"
#include "SystemDefines.hpp"
#include "RocketSM.hpp"
#include "FlightTask.hpp"
#include <map>
#include "Command.hpp"
#include "etl/map.h"


extern TIM_HandleTypeDef htim2;

etl::map<RocketState, BLINK, 10> stateBlinks = etl::map<RocketState, BLINK, 10>{
    {RS_PRELAUNCH, {2, 1000}},
    {RS_FILL, {3, 1000}},
    {RS_ARM, {4, 1000}},
    {RS_IGNITION, {5, 1000}},
    {RS_LAUNCH, {6, 1000}},
    {RS_BURN, {7, 1000}},
    {RS_COAST, {8, 1000}},
    {RS_DESCENT, {9, 1000}},
    {RS_RECOVERY, {10, 1000}},
    {RS_ABORT, {1, 1000}}
};

/**
* @brief Constructor for HDITask
*/
HDITask::HDITask():Task(HDI_TASK_QUEUE_DEPTH_OBJS)
{
}



/**
* @brief Initialize the HDITask
*/
void HDITask::InitTask()
{
    // Make sure the task is not already initialized
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize flight task twice");

    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)HDITask::RunTask,
            (const char*)"HDITask",
            (uint16_t)HDI_TASK_STACK_DEPTH_WORDS,
            (void*)this,
            (UBaseType_t)HDI_TASK_RTOS_PRIORITY,
            (TaskHandle_t*)&rtTaskHandle);

    SOAR_ASSERT(rtValue == pdPASS, "HDITask::InitTask() - xTaskCreate() failed");
}



/**
* @brief Instance Run loop for the Flight Task, runs on scheduler start as long as the task is initialized.
* @param pvParams RTOS Passed void parameters, contains a pointer to the object instance, should not be used
*/
void HDITask::Run(void * pvParams)
{
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
void HDITask::HandleCommand(Command& cm)
{
    //TODO: Since this task will stall for a few milliseconds, we may need a way to eat the whole queue (combine similar eg. REQUEST commands and eat to WDG command etc)
    //TODO: Maybe a HandleEvtQueue instead that takes in the whole queue and eats the whole thing in order of non-blocking to blocking



    //Switch for the GLOBAL_COMMAND
    switch (cm.GetCommand()) {
    case REQUEST_COMMAND: {
        HandleRequestCommand(cm.GetTaskCommand());
        break;
    }
    case TASK_SPECIFIC_COMMAND: {
        break;
    }
    default:
        SOAR_PRINT("HDITask - Received Unsupported Command {%d}\n", cm.GetCommand());
        break;
    }

    cm.Reset();
}

/**
* @brief Handles a Request Command
* @param taskCommand The command to handle
*/
void HDITask::HandleRequestCommand(uint16_t taskCommand)
{
    //Switch for task specific command within DATA_COMMAND
    switch (taskCommand) {
    case PRELAUNCH:
    	SOAR_PRINT("HDI Recieve PreLaunch\n");
        BuzzBlinkSequence(stateBlinks[RS_PRELAUNCH]);
        break;
    case FILL:
        BuzzBlinkSequence(stateBlinks[RS_FILL]);
        break;
    case ARM:
        BuzzBlinkSequence(stateBlinks[RS_ARM]);
        break;
    case IGNITION:
        BuzzBlinkSequence(stateBlinks[RS_IGNITION]);
        break;
    case LAUNCH:
        BuzzBlinkSequence(stateBlinks[RS_LAUNCH]);
        break;
    case BURN:
        BuzzBlinkSequence(stateBlinks[RS_BURN]);
        break;
    case COAST:
        BuzzBlinkSequence(stateBlinks[RS_COAST]);
        break;
    case DESCENT:
        BuzzBlinkSequence(stateBlinks[RS_DESCENT]);
        break;
    case RECOVERY:
        BuzzBlinkSequence(stateBlinks[RS_RECOVERY]);
        break;
    case ABORT:
        BuzzBlinkSequence(stateBlinks[RS_ABORT]);
        break;
    default:
        SOAR_PRINT("UARTTask - Received Unsupported REQUEST_COMMAND {%d}\n", taskCommand);
        break;
    }
}


void HDITask::BuzzBlinkSequence(BLINK blinkSequence){
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    for (uint8_t i = 0; i < blinkSequence.numBlinks; i++) {
        GPIO::LED1::On();
        uint8_t value = 200; // the value for the duty cycle
        htim2.Instance->CCR1 = value;
//
        osDelay(blinkSequence.delayMs);

        GPIO::LED1::Off();
        htim2.Instance->CCR1 = 0;

        osDelay(blinkSequence.delayMs);
    }

}
