#include "Task.hpp"
/**
 ******************************************************************************
 * File Name          : Task.cpp
 * Description        : Task contains the core component base class for all tasks.
 ******************************************************************************
*/

/**
 * @brief Default constructor, instantiates event queue with default size
*/
Task::Task(void)
{
	qEvtQueue = new Queue();
}

/**
 * @brief Constructor with queue depth
 * @param depth Optionally 0, uses the given depth for the event queue
*/
Task::Task(uint16_t depth)
{
	if (depth == 0)
		qEvtQueue = nullptr;
	else
		qEvtQueue = new Queue(depth);
}
