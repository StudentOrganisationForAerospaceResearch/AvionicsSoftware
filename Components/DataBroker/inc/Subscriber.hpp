/**
 ********************************************************************************
 * @file    Subscriber.hpp
 * @author  Shivam Desai
 * @date    Nov 23, 2024
 * @brief
 ********************************************************************************
 */

#ifndef SUBSCRIBER_HPP_
#define SUBSCRIBER_HPP_

/************************************
 * INCLUDES
 ************************************/
#include "Task.hpp"
#include "SystemDefines.hpp"

/************************************
 * MACROS AND DEFINES
 ************************************/

/************************************
 * TYPEDEFS
 ************************************/

/************************************
 * CLASS DEFINITIONS
 ************************************/
class Subscriber {
 public:
  void Init(Task* subscriberTaskHandle) {
    if (taskHandle != nullptr || taskQueue != nullptr) {
      SOAR_ASSERT(false, "You cannot overwrite a subscriber");
      return;
    }
    taskHandle = subscriberTaskHandle;
    taskQueue = taskHandle->GetEventQueue();
  }

  void Delete() {
    taskHandle = nullptr;
    taskQueue = nullptr;
  }

  inline const Task* getSubscriberTaskHandle() const { return taskHandle; }

  inline Queue* getSubscriberQueueHandle() const { return taskQueue; }

 private:
  Task* taskHandle = nullptr;
  Queue* taskQueue = nullptr;
};

/************************************
 * FUNCTION DECLARATIONS
 ************************************/

#endif /* SUBSCRIBER_HPP_ */
