/**
 ********************************************************************************
 * @file    Publisher.hpp
 * @author  Shivam Desai
 * @date    Nov 23, 2024
 * @brief
 ********************************************************************************
 */

#ifndef PUBLISHER_HPP_
#define PUBLISHER_HPP_

/************************************
 * INCLUDES
 ************************************/
#include <DataBrokerMessageTypes.hpp>
#include <stdint.h>
#include <array>
#include "Task.hpp"
#include "Subscriber.hpp"
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
template <typename T, uint8_t MaxSubscribers = 5>
class Publisher {
 public:
  // Constructor
  Publisher(DataBrokerMessageTypes messageType) { publisherMessageType = messageType; }

  // subscribe
  bool Subscribe(Task* taskToSubscribe) {
    // Check if subscriber already exists
    for (Subscriber& subscriber : subscribersList) {
      if (subscriber.getSubscriberTaskHandle() == taskToSubscribe) {
        return true;
      }
    }

    // Add the subscriber
    for (Subscriber& subscriber : subscribersList) {
      if (subscriber.getSubscriberTaskHandle() == nullptr) {
        subscriber.Init(taskToSubscribe);
        return true;
      }
    }

    SOAR_ASSERT(true, "Failed to add subscriber\n");
    return false;
  }

  // unsubscribe
  bool Unsubscribe(Task* taskToUnsubscribe) {
    for (Subscriber& subscriber : subscribersList) {
      if (subscriber.getSubscriberTaskHandle() == taskToUnsubscribe) {
        subscriber.Delete();
        return true;
      }
    }

    SOAR_ASSERT(true, "Subscriber not Deleted\n");
    return false;
  }

  // publish
  void Publish(T* dataToPublish) {
    for (const Subscriber& subscriber : subscribersList) {
      if (subscriber.getSubscriberTaskHandle() != nullptr) {
        // create command
        uint16_t messageType = static_cast<uint16_t>(publisherMessageType);

        Command brokerData(DATA_BROKER_COMMAND, messageType);

        uint8_t* messsageData = reinterpret_cast<uint8_t*>(dataToPublish);

        // copy data to command
        brokerData.CopyDataToCommand(messsageData, sizeof(T));

        subscriber.getSubscriberQueueHandle()->Send(brokerData);
      }
    }
  }

  DataBrokerMessageTypes GetPublisherMessageType() { return publisherMessageType; }

 private:
  // list of subscribers
  Subscriber subscribersList[MaxSubscribers] = {};

  // message type for system routing
  DataBrokerMessageTypes publisherMessageType = DataBrokerMessageTypes::INVALID;
};

/************************************
 * FUNCTION DECLARATIONS
 ************************************/

#endif /* PUBLISHER_HPP_ */
