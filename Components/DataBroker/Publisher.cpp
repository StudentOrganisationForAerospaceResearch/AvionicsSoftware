/**
 ********************************************************************************
 * @file    Publisher.cpp
 * @author  shivam
 * @date    Nov 23, 2024
 * @brief
 ********************************************************************************
 */

/************************************
 * INCLUDES
 ************************************/
#include "Publisher.hpp"
#include "SystemDefines.hpp"
// #include "Command.hpp"

/************************************
 * PRIVATE MACROS AND DEFINES
 ************************************/

/************************************
 * VARIABLES
 ************************************/

/************************************
 * FUNCTION DECLARATIONS
 ************************************/

/************************************
 * FUNCTION DEFINITIONS
 ************************************/

/*****************************************************************
 *****************************************************************/
template <typename T, uint8_t MaxSubscribers>
void Publisher<T, MaxSubscribers>::Subscribe(Task* taskToSubscribe) {
  bool subscriberAdded = false;
  for (Subscriber subscriber : subscribersList) {
    if (subscriber.getSubscriberTaskHandle() == nullptr) {
      subscriber.Init(taskToSubscribe);
      subscriberAdded = true;
    }
  }

  SOAR_ASSERT(subscriberAdded, "Failed to add subscriber");
  return;
}

/*****************************************************************
 *****************************************************************/
template <typename T, uint8_t MaxSubscribers>
void Publisher<T, MaxSubscribers>::Unsubscribe(Task* taskToUnsubscribe) {
  bool subscriberDeleted = false;
  for (Subscriber subscriber : subscribersList) {
    if (subscriber.getSubscriberTaskHandle() == taskToUnsubscribe) {
      subscriber.Delete();
      subscriberDeleted = true;
    }
  }

  SOAR_ASSERT(subscriberDeleted, "Subscriber not Deleted");
}

/*****************************************************************
 *****************************************************************/
template <typename T, uint8_t MaxSubscribers>
void Publisher<T, MaxSubscribers>::Publish(T* dataToPublish) {
  // create command
  Command brokerData(DATA_BROKER_COMMAND, publisherMessageType);

  // copy data to command
  brokerData.CopyDataToCommand(dataToPublish, sizeof(dataToPublish));

  // add command to task queue for all subscribers
  bool messageSent = false;
  for (Subscriber subscriber : subscribersList) {
    if (subscriber.getSubscriberTaskHandle() != nullptr) {
      subscriber.getSubscriberQueueHandle()->Send(brokerData);
    }
  }

  SOAR_ASSERT(messageSent, "The message was sent to no subscribers");
}
