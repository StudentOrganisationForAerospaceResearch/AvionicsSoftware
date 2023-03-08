/**
 ******************************************************************************
 * File Name          : FlashTask.cpp
 * Description        : Flash interface task. Used for logging and state-recovery
 ******************************************************************************
*/
#include "FlashTask.hpp"
#include "GPIO.hpp"
#include "SystemDefines.hpp"
#include "Utils.hpp"
#include "Timer.hpp"
#include "RocketSM.hpp"

/**
 * @brief Constructor for FlashTask
 */
FlashTask::FlashTask() : Task(FLASH_TASK_QUEUE_DEPTH_OBJS)
{
    st_ = nullptr;
}

/**
 * @brief Initialize the FlashTask
 */
void FlashTask::InitTask()
{
    // Make sure the task is not already initialized
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize flash task twice");
    
    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)FlashTask::RunTask,
            (const char*)"FlashTask",
            (uint16_t)FLASH_TASK_STACK_DEPTH_WORDS,
            (void*)this,
            (UBaseType_t)FLASH_TASK_RTOS_PRIORITY,
            (TaskHandle_t*)&rtTaskHandle);
    //st_ = new SystemStorage();

    SOAR_ASSERT(rtValue == pdPASS, "FlashTask::InitTask() - xTaskCreate() failed");

    SOAR_PRINT("Flash Task initialized");
}

/**
 * @brief Instance Run loop for the Flash Task, runs on scheduler start as long as the task is initialized.
 * @param pvParams RTOS Passed void parameters, contains a pointer to the object instance, should not be used
 */
void FlashTask::Run(void * pvParams)
{
    st_ = new SystemStorage();

    while (1) {
        //Process any commands, in non-blocking mode
        Command cm;
        bool res = qEvtQueue->Receive(cm);
        if(res)
            st_->HandleCommand(cm);
    }
}