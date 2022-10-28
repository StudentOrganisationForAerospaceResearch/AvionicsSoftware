/**
 ******************************************************************************
 * File Name          : Queue.hpp
 * Description        :
 *
 *	Queue is a wrapper for RTOS xQueues for use in C++.
 *	with an internal queue handle and queue depth.
 *
 *	Currently only handles Command objects, may want to make this a base template
 *	class for which CommandQueue inherits from.
 ******************************************************************************
*/
#ifndef AVIONICS_INCLUDE_SOAR_CORE_QUEUE_H
#define AVIONICS_INCLUDE_SOAR_CORE_QUEUE_H
/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "Command.hpp"
#include "FreeRTOS.h"
#include "Utils.hpp"

/* Macros --------------------------------------------------------------------*/
#define DEFAULT_QUEUE_SEND_WAIT_TICKS (MS_TO_TICKS(15))	// We wait a max of 15ms to send to a queue

/* Constants -----------------------------------------------------------------*/
//constexpr uint16_t MAX_TICKS_TO_WAIT_SEND = MS_TO_TICKS(1000);

/* Class -----------------------------------------------------------------*/

class Queue {
public:
	//Constructors
	Queue(void);
	Queue(uint16_t depth);

	//Functions
	bool Send(Command& command);
	bool SendFromISR(Command& command);

	bool SendToFront(Command& command);

	bool Receive(Command& cm, uint32_t timeout_ms = 0);
	bool ReceiveWait(Command& cm); //Blocks until a command is received

	//Getters
	uint16_t GetQueueMessageCount() const { return uxQueueMessagesWaiting(rtQueueHandle); }
	uint16_t GetQueueDepth() const { return queueDepth; }

protected:
	//RTOS
	QueueHandle_t rtQueueHandle;	// RTOS Event Queue Handle
	
	//Data
	uint16_t queueDepth;			// Max queue depth
};

#endif /* AVIONICS_INCLUDE_SOAR_CORE_QUEUE_H */