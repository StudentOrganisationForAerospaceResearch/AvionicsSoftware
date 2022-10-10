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
 * @brief Sends a command object to the queue
 * @param command Command object reference to send
 * @return true on success, false on failure (queue full)
*/
bool Queue::Send(Command& command)
{
	if (xQueueSend(rtQueueHandle, &command, DEFAULT_QUEUE_SEND_WAIT_TICKS) == pdPASS)
		return true;

	//TODO: Probably should have a debug message printing here if we're getting a queue full error

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
