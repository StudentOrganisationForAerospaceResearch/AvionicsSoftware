/**
 ******************************************************************************
 * File Name          : FlightTask.cpp
 * Description        : Primary flight task, default task for the system.
 ******************************************************************************
*/
#include "GPIO.hpp"
#include "SystemDefines.hpp"
#include "Timer.hpp"
#include "WatchdogTask.hpp"

void WatchdogTask::HeartbeatFailureCallback(TimerHandle_t rtTimerHandle)
{

	SOAR_PRINT("The system lost its hearbeat and had to reset!!!");
	// SOAR_ASSERT("The system lost its hearbeat and had to reset!!!");
	Timer::DefaultCallback(rtTimerHandle);
}



/**
 * @brief Instance Run loop for the Watchdog Task, runs on scheduler start as long as the task is initialized.
 * @param pvParams RTOS Passed void parameters, contains a pointer to the object instance, should not be used
 */
void WatchdogTask::Run(void * pvParams)
{
    uint32_t tempSecondCounter = 0; // TODO: Temporary counter, would normally be in HeartBeat task or HID Task, unless FlightTask is the HeartBeat task
    GPIO::LED1::Off();

    static Timer HeartbeatPeriod(HeartbeatFailureCallback);

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
        GPIO::LED2::On();
        GPIO::LED3::On();
        osDelay(500);
        GPIO::LED1::Off();
        GPIO::LED2::Off();
        GPIO::LED3::Off();
        osDelay(500);

        //Every cycle, print something out (for testing)
        SOAR_PRINT("FlightTask::Run() - [%d] Seconds\n", tempSecondCounter++);


        // TODO: Message beeps
    }
}
