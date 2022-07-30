/**
 ******************************************************************************
 * File Name          : Task.h
 * Description        : Task contains the core component base class for all tasks.
 ******************************************************************************
*/
#ifndef AVIONICS_INCLUDE_SOAR_CORE_TASK_H
#define AVIONICS_INCLUDE_SOAR_CORE_TASK_H
/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "Queue.hpp"

/* Macros --------------------------------------------------------------------*/

/* Enums -----------------------------------------------------------------*/

/* Class -----------------------------------------------------------------*/

class Task {
public:
	//Constructors
	Task(void);
	Task(uint16_t depth);

	Queue* GetEventQueue() const { return qEvtQueue; }

protected:
	//RTOS
	TaskHandle_t rtTaskHandle;		// RTOS Task Handle

	//Task structures
	Queue* qEvtQueue;	// Task event queue
};

#endif /* AVIONICS_INCLUDE_SOAR_CORE_TASK_H */