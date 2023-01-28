/**
 ******************************************************************************
 * File Name          : Mutex.hpp
 * Description        : Mutex is an object wrapper for rtos mutexes.
 ******************************************************************************
*/
#ifndef AVIONICS_INCLUDE_SOAR_CORE_MUTEX_H
#define AVIONICS_INCLUDE_SOAR_CORE_MUTEX_H
/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"

/* Macros --------------------------------------------------------------------*/


/* Class -----------------------------------------------------------------*/

/**
 * @brief Mutex class is a wrapper for rtos mutexes.
 */
class Mutex
{
public:
    // Constructors / Destructor
    Mutex();
    ~Mutex();

    // Public functions
    bool Lock(uint32_t timeout_ms = portMAX_DELAY);
    bool Unlock();

private:
    SemaphoreHandle_t rtSemaphoreHandle;

};


#endif /* AVIONICS_INCLUDE_SOAR_CORE_MUTEX_H */