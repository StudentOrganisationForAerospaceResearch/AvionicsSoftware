/**
 ******************************************************************************
 * File Name          : FlightTask.cpp
 * Description        : Primary flight task, default task for the system.
 ******************************************************************************
*/
#include "FlightTask.hpp"
#include "GPIO.hpp"
#include "SystemDefines.hpp"
#include "Timer.hpp"

void test_timer_state () {
    Timer testTimer1;
    SOAR_PRINT("Expected Output: 0 (UNINITIALIZED)\n");
    SOAR_PRINT("The current timer state is: %d\n\n", testTimer1.GetState());

    SOAR_PRINT("Starting Timer...\n");
    testTimer1.StartTimer();
    SOAR_PRINT("Expected Output: 1 (COUNTING)\n");
	SOAR_PRINT("The current timer state is: %d\n\n", testTimer1.GetState());

	SOAR_PRINT("Stopping Timer...\n");
	testTimer1.StopTimer();
	SOAR_PRINT("Expected Output: 2 (PAUSED)\n");
	SOAR_PRINT("The current timer state is: %d\n\n", testTimer1.GetState());

	osDelay(1100);
	SOAR_PRINT("Expected Output: 3 (COMPLETE)\n");
	SOAR_PRINT("The current timer state is: %d\n\n", testTimer1.GetState());

	SOAR_PRINT("'TIMER HAS BEEN DELETED' should be printed:  \n");
}

void test_destructor() {
	SOAR_PRINT("\n Testing destructor...\n");
	SOAR_PRINT("Expected Output: 'TIMER HAS BEEN DELETED'\n");
	{
		Timer testTimer2;
	}
}

void test_period () {
	Timer testTimer3;

	testTimer3.StartTimer();
	SOAR_PRINT("\n Testing GetPeriod function...\n");
	SOAR_PRINT("Expected Output: 1000 ms\n");
	SOAR_PRINT("Actual Output: %d\n\n", testTimer3.GetPeriod());

	SOAR_PRINT("Changing Period to 5000ms\n");
	SOAR_PRINT("Expected Result: 5000ms\n");
	SOAR_PRINT("Actual Result: %d\n\n", testTimer3.GetPeriod());

	SOAR_PRINT("Testing GetState function...\n");
	SOAR_PRINT("Expected Output: 2 (PAUSED)\n");
	SOAR_PRINT("Actual Output: %d\n\n", testTimer3.GetState());

	SOAR_PRINT("Testing to see if the timer restarts if timer period is changed\n");
	SOAR_PRINT("Using GetPeriod function ...\n");
	SOAR_PRINT("Expected Output: 5000 ms\n");
	SOAR_PRINT("Actual Output: %d\n\n", testTimer3.GetRemainingTime());

	SOAR_PRINT("Testing ChangePeriodAndStart function...\n");
	testTimer3.ChangePeriodAndStart(7000);
	SOAR_PRINT("Expected Output: 7000 ms\n");
	SOAR_PRINT("Actual Output: %d\n\n", testTimer3.GetPeriod());

	SOAR_PRINT("Testing GetState function...\n");
	SOAR_PRINT("Expected Output: 1 (COUNTING)\n");
	SOAR_PRINT("Actual Output: %d\n\n", testTimer3.GetState());

	SOAR_PRINT("'TIMER HAS BEEN DELETED' should be printed:  \n");
}

void test_autoreload () {
	Timer testTimer4;

	SOAR_PRINT("\n Testing GetAutoReload function...\n");
	SOAR_PRINT("Expected Output: 'Timer is set to autoreload'\n");
	if (testTimer4.GetAutoReload() == true) {
		SOAR_PRINT("Actual Output : 'Timer is set to autoreload'\n\n");
	}
	else {
		SOAR_PRINT("ERROR OCCURRED!!!\n\n");
	}

	SOAR_PRINT("Changing timer to one-shot...\n");
	testTimer4.SetAutoReload(false);
	SOAR_PRINT("Expected Output: 'Timer is set to one-shot'\n");
	if (testTimer4.GetAutoReload() == false) {
		SOAR_PRINT("Actual Output : 'Timer is set to one-shot'\n\n");
	}
	else {
		SOAR_PRINT("ERROR OCCURRED!!!\n\n");
	}

	SOAR_PRINT("'TIMER HAS BEEN DELETED' should be printed:  \n");
}

void test_reset(){
	Timer testTimer5;
	SOAR_PRINT("\n Trying to reset timer in UNINITIALIZED state\n");
	SOAR_PRINT("Expected Output: Timer did not reset\n");
	if (testTimer5.ResetTimer() == false) {
		SOAR_PRINT("Actual Output: Timer did not reset\n\n");
	}
	else {
		SOAR_PRINT("ERROR OCCURRED!!!\n\n");
	}

	testTimer5.StartTimer();
	osDelay(300);
	SOAR_PRINT("Testing reset function...\n");
	testTimer5.ResetTimer();
	SOAR_PRINT("Expected Result: Timer Reset Successfully\n");
	if (testTimer5.ResetTimer() == true) {
		SOAR_PRINT("Actual Output: Timer Reset Successfully\n\n");
	}
	else {
		SOAR_PRINT("ERROR OCCURRED!!!\n\n");
	}

	SOAR_PRINT("Testing GetState function...\n");
	SOAR_PRINT("Expected Output: 2 (PAUSED)\n");
	SOAR_PRINT("Actual Output: %d\n\n", testTimer5.GetState());

	SOAR_PRINT("Testing ResetTimerAndStart function..\n");
	if (testTimer5.ResetTimerAndStart() == true) {
		SOAR_PRINT("Actual Output: Timer Reset Successfully\n\n");
	}
	else {
		SOAR_PRINT("ERROR OCCURRED!!!\n\n");
	}

	SOAR_PRINT("Testing GetState function...\n");
	SOAR_PRINT("Expected Output: 1 (COUNTING)\n");
	SOAR_PRINT("Actual Output: %d\n\n", testTimer5.GetState());

	SOAR_PRINT("'TIMER HAS BEEN DELETED' should be printed:  \n");
}

void test_get_time () {
	Timer testTimer6;

	SOAR_PRINT("Testing GetRemainingTime function ...\n");
	SOAR_PRINT("Expected Output: 1000 ms\n");
	SOAR_PRINT("Actual Output: %d ms \n\n", testTimer6.GetRemainingTime());

	osDelay(300);
	SOAR_PRINT("Expected Output: 700 ms\n");
	SOAR_PRINT("Actual Output: %d ms \n\n", testTimer6.GetRemainingTime());

	osDelay(300);
	testTimer6.StopTimer();
	SOAR_PRINT("Expected Output: 400 ms\n");
	SOAR_PRINT("Actual Output: %d ms \n\n", testTimer6.GetRemainingTime());

	osDelay(500);
	SOAR_PRINT("Expected Output: 0 ms\n");
	SOAR_PRINT("Actual Output: %d ms \n\n", testTimer6.GetRemainingTime());

	SOAR_PRINT("'TIMER HAS BEEN DELETED' should be printed:  \n");
}

//TODO: Consider moving all InitTask functions to the bottom of the cpp file, since it shouldn't need to be changed that much
void FlightTask::InitTask()
{
	// Make sure the task is not already initialized
	SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize flight task twice");
	
	BaseType_t rtValue =
		xTaskCreate((TaskFunction_t)FlightTask::RunTask,
			(const char*)"FlightTask",
			(uint16_t)FLIGHT_TASK_STACK_DEPTH_WORDS,
			(void*)this,
			(UBaseType_t)FLIGHT_TASK_RTOS_PRIORITY,
			(TaskHandle_t*)&rtTaskHandle);

	SOAR_ASSERT(rtValue == pdPASS, "FlightTask::InitTask() - xTaskCreate() failed");
}

/**
 * @brief Instance Run loop for the Flight Task, runs on scheduler start as long as the task is initialized.
 * @param pvParams RTOS Passed void parameters, contains a pointer to the object instance, should not be used
 */
void FlightTask::Run(void * pvParams)
{
	GPIO::LED1::Off();

    test_timer_state();
    test_destructor();
    test_period ();
    test_autoreload ();
    test_reset();
    test_get_time ();

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
