/**
******************************************************************************
* File Name          : HDITask.cpp
* Description        : Human Device Interface Task. Will give a sequence command
* 					   to LED and buzzer to tell what state it is.
******************************************************************************
*/
#include "HDITask.hpp"
#include "Command.hpp"
#include "FlightTask.hpp"
#include "GPIO.hpp"
#include "RocketSM.hpp"
#include "SystemDefines.hpp"
#include "etl/map.h"

extern TIM_HandleTypeDef htim2;
constexpr uint16_t BUZZER_DEFAULT_DUTY_CYCLE = 190;

/* One cycle is defined as a certain amount of beeps for that state.
 * At RS_TEST, 1 cycle will have 1 beep that lasts 500 milliseconds, followed by 500 milliseconds of silence.
 * Before the next cycle begins, there will be 1000 milliseconds of silence to be able to distinguish that
 * the next cycle will begin.
 * Again to clarify, for RS_PRELAUNCH, it will beep twice where each beep will last 500 milliseconds, followed
 * by 500 milliseconds of silence between each of the beep.
 * This is the same pattern for all different states but with the beeps and silence lasting the specified amount
 * in the etl map.
 * At RS_IGNITION will have a beep that last 300 milliseconds, followed by 300 milliseconds of silence. This is the same
 * for RS_LAUNCH but it will have 2 beeps each lasting 300 milliseconds and 300 milliseconds of silence between them
 * RS_DESCENT and RS_RECOVERY will have 5 and 6 beeps (respectively) that each last 180 milliseconds
 * RS_ABORT will be easily distinguishable as it will have 7 beeps that each last 100 milliseconds and 100 milliseconds
 * of silence between each one.
 *
 * */

etl::map<RocketState, HDIConfig, 11> stateBlinks =
    etl::map<RocketState, HDIConfig, 11>{
        {RS_TEST, {1, 500}},     {RS_PRELAUNCH, {2, 500}},
        {RS_FILL, {3, 500}},     {RS_ARM, {4, 500}},
        {RS_IGNITION, {1, 300}}, {RS_LAUNCH, {2, 300}},
        {RS_BURN, {3, 300}},     {RS_COAST, {4, 300}},
        {RS_DESCENT, {5, 180}},  {RS_RECOVERY, {6, 180}},
        {RS_ABORT, {7, 100}}};

/**
* @brief Constructor for HDITask
*/
HDITask::HDITask() : Task(HDI_TASK_QUEUE_DEPTH_OBJS), buzzerMuted_(false) {}

/**
* @brief Initialize the HDITask
*/
void HDITask::InitTask() {
  // Make sure the task is not already initialized
  SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize HDI task twice");

  BaseType_t rtValue = xTaskCreate(
      (TaskFunction_t)HDITask::RunTask, (const char*)"HDITask",
      (uint16_t)HDI_TASK_STACK_DEPTH_WORDS, (void*)this,
      (UBaseType_t)HDI_TASK_RTOS_PRIORITY, (TaskHandle_t*)&rtTaskHandle);

  SOAR_ASSERT(rtValue == pdPASS, "HDITask::InitTask() - xTaskCreate() failed");
}

/**
* @brief Instance Run loop for the Flight Task, runs on scheduler start as long as the task is initialized.
* @param pvParams RTOS Passed void parameters, contains a pointer to the object instance, should not be used
*/
void HDITask::Run(void* pvParams) {

  while (1) {
    htim2.Instance->CCR1 = BUZZER_DEFAULT_DUTY_CYCLE;

    Command cm;

    //Wait forever for a command,
    //look forward for anything in queue, if there is , if there is anything update index, if there isn't anything do what was done before
    //state field, last known state
    if (qEvtQueue->Receive(cm, 1000)) {
      HandleCommand(cm);
    } else {
      BuzzBlinkSequence(currentConfig);
    }
    //Process the command
  }
}

/**
* @brief Handles a command
* @param cm Command reference to handle
*/
void HDITask::HandleCommand(Command& cm) {
  //TODO: Since this task will stall for a few milliseconds, we may need a way to eat the whole queue (combine similar eg. REQUEST commands and eat to WDG command etc)
  //TODO: Maybe a HandleEvtQueue instead that takes in the whole queue and eats the whole thing in order of non-blocking to blocking

  //Switch for the GLOBAL_COMMAND
  switch (cm.GetCommand()) {
    case REQUEST_COMMAND: {
      HandleRequestCommand(cm.GetTaskCommand());
      break;
    }
    case TASK_SPECIFIC_COMMAND: {
      if (HDITaskCommands::MUTE == cm.GetTaskCommand()) {
        buzzerMuted_ = true;
      } else if (HDITaskCommands::UNMUTE == cm.GetTaskCommand()) {
        buzzerMuted_ = false;
      }
      break;
    }
    default:
      SOAR_PRINT("HDITask - Received Unsupported Command {%d}\n",
                 cm.GetCommand());
      break;
  }

  cm.Reset();
}

/**
* @brief Handles a Request Command
* @param taskCommand The command to handle
*/
void HDITask::HandleRequestCommand(uint16_t taskCommand) {
  //Switch for task specific command within DATA_COMMAND
  if ((RocketState)taskCommand >= RS_PRELAUNCH &&
      (RocketState)taskCommand < RS_NONE) {
    auto it = stateBlinks.find((RocketState)taskCommand);
    if (it != stateBlinks.end()) {
      currentConfig = it->second;
      BuzzBlinkSequence(currentConfig);
    }
  }
}

void HDITask::BuzzBlinkSequence(HDIConfig blinkSequence) {
  for (uint8_t i = 0; i < blinkSequence.numBlinks; i++) {
    // Start the buzzer
    if (!buzzerMuted_) {
      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    }

    // Turn on the LED and play the beep
    GPIO::LED1::On();
    osDelay(blinkSequence.delayMs);  // a beep lasts this long

    // Stop the buzzer and turn off the LED
    if (!buzzerMuted_) {
      HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    }
    GPIO::LED1::Off();

    // Wait for the silence duration between beeps
    osDelay(blinkSequence.delayMs);
  }
}
