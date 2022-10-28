/**
 ******************************************************************************
 * File Name          : Queue.cpp
 * Description        : Implementation of the Queue class
 ******************************************************************************
*/
#include "Queue.hpp"
#include "Command.hpp"
#include "SystemDefines.hpp"
#include "FreeRTOS.h"


/**
 * @brief Constructor for the Queue class uses DEFAULT_QUEUE_SIZE for queue depth
*/
Queue::Queue(void)
{
	//Initialize RTOS Queue handle
	rtQueueHandle = xQueueCreate(DEFAULT_QUEUE_SIZE, sizeof(Command));
	queueDepth = 0;
}

/**
 * @brief Constructor with depth for the Queue class
 * @param depth Queue depth
*/
Queue::Queue(uint16_t depth)
{
	//Initialize RTOS Queue handle with given depth
	rtQueueHandle = xQueueCreate(depth, sizeof(Command));
	queueDepth = 0;
}

/**
 * @brief Sends a command object to the queue, safe to call from ISR
 * @param command Command object reference to send
 * @return true on success, false on failure (queue full)
*/
bool Queue::SendFromISR(Command& command)
{
	//Note: There NULL param here could be used to wake a task right after after exiting the ISR
	if (xQueueSendFromISR(rtQueueHandle, &command, NULL) == pdPASS)
		return true;

	command.Reset();

	return false;
}

/**
 * @brief Sends a command object to the front of the queue, use for high priority commands
 * @param command Command object reference to send
 * @return true on success, false on failure (queue full)
 */
bool Queue::SendToFront(Command& command)
{
	//Send to the back of the queue
	if (xQueueSendToFront(rtQueueHandle, &command, DEFAULT_QUEUE_SEND_WAIT_TICKS) == pdPASS)
		return true;

	SOAR_PRINT("Could not send data to front of queue!");
	command.Reset();

	return false;
}

/**
 * @brief Sends a command object to the queue (sends to back of queue in FIFO order)
 * @param command Command object reference to send
 * @return true on success, false on failure (queue full)
*/
bool Queue::Send(Command& command)
{
	if (xQueueSend(rtQueueHandle, &command, DEFAULT_QUEUE_SEND_WAIT_TICKS) == pdPASS)
		return true;

	//TODO: It may be possible to have this automatically set the command to not free data externally as we've "passed" control of the data over, which might let us use a destructor to free the data

	SOAR_PRINT("Could not send data to queue!");
	command.Reset();

	return false;
}

/**
 * @brief Polls queue with specific timeout, blocks for timeout_ms, returns null on no data
 * @param timeout_ms Time to block for
 * @param cm Command object to copy received data into
 * @return TRUE if we received a command, FALSE otherwise
*/
bool Queue::Receive(Command& cm, uint32_t timeout_ms)
{
	if(xQueueReceive(rtQueueHandle, &cm, MS_TO_TICKS(timeout_ms)) == pdTRUE) {
		return true;
	}
	return false;
}

/**
 * @brief Polls queue with specific timeout, blocks forever
 * @param cm Command object to copy received data into
 * @return TRUE if we received a command, FALSE otherwise (should rarely return false)
*/
bool Queue::ReceiveWait(Command& cm)
{
	if (xQueueReceive(rtQueueHandle, &cm, HAL_MAX_DELAY) == pdTRUE) {
		return true;
	}
	return false;
}
