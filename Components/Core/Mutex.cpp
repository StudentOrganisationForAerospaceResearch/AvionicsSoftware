/**
 ******************************************************************************
 * File Name          : Mutex.cpp
 * Description        : This file provides all the Mutex functionality.
 ******************************************************************************
*/

#include "Mutex.hpp"
#include "SystemDefines.hpp"
#include "Utils.hpp"

/**
 * @brief Constructor for the Mutex class.
 */
Mutex::Mutex()
{
	rtSemaphoreHandle = xSemaphoreCreateMutex();

	SOAR_ASSERT(rtSemaphoreHandle != NULL, "Semaphore creation failed.");
}


/**
 * @brief Destructor for the Mutex class.
 */
Mutex::~Mutex()
{
	vSemaphoreDelete(rtSemaphoreHandle);
}

/**
 * @brief This function is used to lock the Mutex.
 * @param timeout_ms The time to wait for the Mutex before it fails. If timeout_ms is not provided, the function will wait indefinitely.
 * @return True on success, false on failure.
*/
bool Mutex::Lock(uint32_t timeout_ms)
{
	return xSemaphoreTake(rtSemaphoreHandle, MS_TO_TICKS(timeout_ms)) == pdTRUE;
}


/**
 * @brief This function will attempt to unlock the mutex
 * @return True on success (mutex unlocked) false in failure (mutex was not unlocked)
*/
bool Mutex::Unlock()
{
	return xSemaphoreGive(rtSemaphoreHandle) == pdTRUE;
}

