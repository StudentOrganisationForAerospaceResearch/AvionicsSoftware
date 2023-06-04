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

extern TIM_HandleTypeDef htim2;

std::map <RocketState, BLINK> stateBlinks = {
    {RS_PRELAUNCH, {2, 1000}},
    {RS_FILL,{3, 1000}},
    {RS_ARM, {4, 1000}},
    {RS_IGNITION, {5, 1000}},
    {RS_LAUNCH, {6, 1000}},
    {RS_BURN, {7, 1000}},
    {RS_COAST, {8, 1000}},
    {RS_DESCENT,{9, 1000}},
    {RS_RECOVERY, {10, 1000}},
    {RS_ABORT,{1, 100}}
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
        SOAR_PRINT("Run task HDI");
    }
}


void HDITask::BuzzBlinkSequence(BLINK blinkSequence){
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    for (uint8_t i = 0; i < blinkSequence.numBlinks; i++) {
        GPIO::LED1::On();
        uint8_t value = 0; // the value for the duty cycle
        while (value<210){
            htim2.Instance->CCR1 = value; // vary the duty cycle
            value += 20; // increase the duty cycle by 20
            osDelay (500); // wait for 500 ms
        }
        value = 0;



        SOAR_PRINT("LED PLS");
        osDelay(blinkSequence.delayMs);



        GPIO::LED1::Off();
        SOAR_PRINT("LED OFF");
        //BUZZER_OFF();
        osDelay(blinkSequence.delayMs);
    }
}
