/**
 ******************************************************************************
 * File Name          : FlightTask.cpp
 * Description        : Primary flight task, default task for the system.
 ******************************************************************************
*/
#include "FlightTask.hpp"
#include "GPIO.hpp"
#include "SystemDefines.hpp"

void FlightTask::InitTask()
{
	BaseType_t rtValue =
		xTaskCreate((TaskFunction_t)FlightTask::Run,
			(const char*)"FlightTask",
			(uint16_t)FLIGHT_TASK_STACK_SIZE,
			(void*)this,
			(UBaseType_t)FLIGHT_TASK_PRIORITY,
			(TaskHandle_t*)&rtTaskHandle);

	SOAR_ASSERT(rtValue == pdPASS, "FlightTask::InitTask() - xTaskCreate() failed");
}

//TODO: Need to convert to UARTTask style RunTask and Inst Run() to enable Run() to have direct access to inst variables without constantly calling Inst()
void FlightTask::Run(void * pvParams)
{
	GPIO::LED1::Off();

	while (1) {
		// There's effectively 3 types of tasks... 'Async' and 'Synchronous-Blocking' and 'Synchronous-Non-Blocking'
		// Asynchronous tasks don't require a fixed-delay and can simply delay using xQueueReceive, it will immedietly run the next task
		// cycle as soon as it gets an event.

		// Synchronous-Non-Blocking tasks require a fixed-delay and will require something like an RTOS timer that tracks the time till the next run cycle,
		// and will delay using xQueueReceive for the set time, but if it gets interrupted by an event will handle the event then restart a xQueueReceive with
		// the time remaining in the timer

		// Synchronous-Blocking tasks are simpler to implement, they do NOT require instant handling of queue events, and will simply delay using osDelay() and
		// poll the event queue once every cycle.

		// This task below with the display would be a 'Synchronous-Non-Blocking' we want to handle queue events instantly, but keep a fixed delay
		// Could consider a universal queue that directs and handles commands to specific tasks, and a task that handles the queue events and then calls the
		// Mappings between X command and P subscribers (tasks that are expecting it).

		// Since FlightTask is so critical to managing the system, it may make sense to make this a Async task that handles commands as they come in, and have these display commands be routed over to the DisplayTask
		// or maybe HID (Human Interface Device) task that handles both updating buzzer frequencies and LED states.
		GPIO::LED1::On();
		osDelay(500);
		GPIO::LED1::Off();
		osDelay(500);

		//Every cycle, print something out (for testing)
		SOAR_PRINT("FlightTask::Run() - Running\n");

		//osDelay(FLIGHT_PHASE_DISPLAY_FREQ);

		//// Half the buzzer frequency for flight phase beeps
		//// (slightly less important, and only a bit quieter)
		//htim2.Init.Prescaler = ((htim2.Init.Prescaler + 1) * 2) - 1;
		//if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		//	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		//	osDelay(BUZZER_ERR_PERIOD);
		//	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
		//}

		//// Beep n times for flight phase n, and blink LED 1
		//for (int i = -1; i < getCurrentFlightPhase(); i++) {
		//	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		//	HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, 1);
		//	osDelay(FLIGHT_PHASE_BLINK_FREQ);

		//	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
		//	HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, 0);
		//	osDelay(FLIGHT_PHASE_BLINK_FREQ);
		//}

		//// Return the buzzer to its optimal frequency for message beeps
		//htim2.Init.Prescaler = ((htim2.Init.Prescaler + 1) / 2) - 1;
		//if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		//	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		//	osDelay(BUZZER_ERR_PERIOD);
		//	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
		//}

		// TODO: Message beeps
	}
}
