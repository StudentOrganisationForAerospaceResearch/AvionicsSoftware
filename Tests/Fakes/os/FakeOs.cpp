
#include "FakeOSTypes.hpp"

/* Variables ---------------------------------------------------------------- */

/* Functions ---------------------------------------------------------------- */

UBaseType_t uxQueueMessagesWaiting(const QueueHandle_t xQueue) { return 0; }

size_t xPortGetFreeHeapSize() { return 0; }

size_t xPortGetMinimumEverFreeHeapSize() { return 0; }

osStatus osKernelStart() { return osOK; }

osStatus osDelay(uint32_t millisec) { return osOK; }

void vTaskSuspendAll() {}

BaseType_t xTaskResumeAll() {
  return 0;
}
