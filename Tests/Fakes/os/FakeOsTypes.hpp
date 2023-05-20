
#ifndef FAKEOSTYPES_HPP
#define FAKEOSTYPES_HPP

#include <stdint.h>

#include <cstddef>

/* Constants ---------------------------------------------------------------- */

#define ARBITRARY_VALUE 1000

#define portMAX_DELAY ARBITRARY_VALUE
#define osKernelSysTickFrequency ARBITRARY_VALUE

/* Types -------------------------------------------------------------------- */

typedef enum {
  osOK = 0,
  osEventSignal = 0x08,
  osEventMessage = 0x10,
  osEventMail = 0x20,
  osEventTimeout = 0x40,
  osErrorParameter = 0x80,
  osErrorResource = 0x81,
  osErrorTimeoutResource = 0xC1,
  osErrorISR = 0x82,
  osErrorISRRecursive = 0x83,
  osErrorPriority = 0x84,
  osErrorNoMemory = 0x85,
  osErrorValue = 0x86,
  osErrorOS = 0xFF,
  os_status_reserved = 0x7FFFFFFF
} osStatus;

typedef struct {
  uint8_t fakeVal;
} SemaphoreHandle_t;

typedef struct {
  uint8_t fakeVal;
} QueueHandle_t;

typedef struct {
  uint8_t fakeVal;
} TaskHandle_t;

typedef struct {
  uint8_t fakeVal;
} TimerHandle_t;

typedef unsigned long UBaseType_t;
typedef long BaseType_t;
typedef SemaphoreHandle_t osMutexId;

/* Declarations ------------------------------------------------------------- */

UBaseType_t uxQueueMessagesWaiting(const QueueHandle_t xQueue);

size_t xPortGetFreeHeapSize();

size_t xPortGetMinimumEverFreeHeapSize();

osStatus osKernelStart();

osStatus osDelay(uint32_t millisec);

void vTaskSuspendAll();

BaseType_t xTaskResumeAll();

#endif