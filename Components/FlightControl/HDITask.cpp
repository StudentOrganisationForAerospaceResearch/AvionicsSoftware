/**
******************************************************************************
* File Name          : HDITask.cpp
* Description        : Human Device Interface Task. Will give a sequence command
* 						to LED and buzzer to tell what state it is.
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
constexpr uint16_t BUZZER_DEFAULT_DUTY_CYCLE = 200;

etl::map<RocketState, HDIConfig, 11> stateBlinks = etl::map<RocketState, HDIConfig, 11>{
	{RS_TEST, {1, 500}},
	{RS_PRELAUNCH, {2, 500}},
    {RS_FILL, {3, 500}},
    {RS_ARM, {4, 500}},
    {RS_IGNITION, {1, 300}},
    {RS_LAUNCH, {2, 300}},
    {RS_BURN, {3, 300}},
    {RS_COAST, {4, 300}},
    {RS_DESCENT, {5, 150}},
    {RS_RECOVERY, {6, 150}},
    {RS_ABORT, {7, 100}}
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
		htim2.Instance->CCR1 = BUZZER_DEFAULT_DUTY_CYCLE;

		Command cm;

		//Wait forever for a command,
		//look forward for anything in queue, if there is , if there is anything update index, if there isn't anything do what was done before
		//state field, last known state
		if(qEvtQueue->Receive(cm, 1000)){
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
	if((RocketState)taskCommand >= RS_PRELAUNCH && (RocketState)taskCommand< RS_NONE){
		BuzzBlinkSequence(currentConfig);
		currentConfig = stateBlinks[(RocketState)taskCommand];
		if ((RocketState)taskCommand == RS_ABORT){
			SOAR_PRINT("THIS IS RS_ABORT!!! TIPEE");
		}
		if ((RocketState)taskCommand == RS_PRELAUNCH){
			SOAR_PRINT("THIS IS RS_PRELAUNCH!!! TIPEE");
		}
		if ((RocketState)taskCommand == RS_FILL){
			SOAR_PRINT("THIS IS RS_FILL!!! TIPEE");
		}
	}
}


void HDITask::BuzzBlinkSequence(HDIConfig blinkSequence)
{
    for (uint8_t i = 0; i < blinkSequence.numBlinks; i++)
    {
        // Start the buzzer
    	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

        // Turn on the LED
        GPIO::LED1::On();

        // Play the beep
        osDelay(blinkSequence.delayMs); //beep last 0.5 seconds

        // Stop the buzzer
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

        // Turn off the LED
        GPIO::LED1::Off();

        // Wait for the silence duration between beeps
		osDelay(blinkSequence.delayMs);

    }
    //osDelay(8000); //wait 8 seconds before the next state feedback is given


}
