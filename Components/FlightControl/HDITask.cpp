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
#include "Command.hpp"
#include "etl/map.h"


extern TIM_HandleTypeDef htim2;

etl::map<RocketState, HDIConfig, 11> stateBlinks = etl::map<RocketState, HDIConfig, 11>{
	{RS_TEST, {1, 500}},
	{RS_PRELAUNCH, {2, 500}},
    {RS_FILL, {3, 500}},
    {RS_ARM, {4, 500}},
    {RS_IGNITION, {5, 500}},
    {RS_LAUNCH, {6, 500}},
    {RS_BURN, {7, 500}},
    {RS_COAST, {8, 500}},
    {RS_DESCENT, {9, 500}},
    {RS_RECOVERY, {10, 500}},
    {RS_ABORT, {11, 500}}
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
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize HDI task twice");

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
		uint8_t value = 200; // the value for the duty cycle
		htim2.Instance->CCR1 = value;

		Command cm;

		//Wait forever for a command,
		//look forward for anything in queue, if there is , if there is anything update index, if there isn't anything do what was done before
		//state field, last known state
		if(qEvtQueue->Receive(cm)){
			HandleCommand(cm);
		}
		else{
			BuzzBlinkSequence(currentConfig);
		}
		//Process the command
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
	currentConfig = stateBlinks[RS_ABORT];
	if((RocketState)taskCommand >= RS_PRELAUNCH && (RocketState)taskCommand< RS_NONE){
		BuzzBlinkSequence(stateBlinks[(RocketState)taskCommand]);
		currentConfig = stateBlinks[(RocketState)taskCommand];
	}
}


void HDITask::BuzzBlinkSequence(HDIConfig blinkSequence)
{
    const uint8_t NUM_BEEPS = blinkSequence.numBlinks;

    for (uint8_t i = 0; i < NUM_BEEPS; i++)
    {
        // Start the buzzer
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

        // Turn on the LED
        GPIO::LED1::On();

        // Play the beep
        osDelay(500); //beep last 0.5 seconds

        // Stop the buzzer
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

        // Turn off the LED
        GPIO::LED1::Off();

        // Wait for the silence duration between beeps
		osDelay(500); //no beep for 0.5 seconds or in other words, next beep will happen in 0.5 seconds

    }
    osDelay(8000); //wait 8 seconds before the next state feedback is given


}
