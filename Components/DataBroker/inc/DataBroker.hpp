/**
 ********************************************************************************
 * @file    DataBroker.hpp
 * @author  Shivam Desai
 * @date    Nov 23, 2024
 * @brief
 ********************************************************************************
 */

#ifndef DATA_BROKER_HPP_
#define DATA_BROKER_HPP_

/************************************
 * INCLUDES
 ************************************/
#include "Publisher.hpp"
#include "SensorDataTypes.hpp"
#include "Command.hpp"
#include "DataBrokerMessageTypes.hpp"
#include "SystemDefines.hpp"
#include "Mutex.hpp"
#include <type_traits>
#include <cstring>
#include <string>
#include <sstream>

/************************************
 * MACROS AND DEFINES
 ************************************/

/************************************
 * TYPEDEFS
 ************************************/

/************************************
 * CLASS DEFINITIONS
 ************************************/
class DataBroker {
 public:
  /**
   * @brief Publish data of a certain type
   *         NOTE: You must ensure that there is a publisher for that type
   */
  template <typename T>
  static void Publish(T* dataToPublish) {
    if (subscriberListLock.Lock(SUBSCRIBER_LIST_MUTEX_TIMEOUT)) {
      Publisher<T>* publisher = getPublisher<T>();
      if (publisher != nullptr) {
        publisher->Publish(dataToPublish);
      } else {
        SOAR_ASSERT("Data Publisher not found \n");
      }
      subscriberListLock.Unlock();
      return;
    } else {
      SOAR_PRINT("Could Not Subscribe to Data Broker Publisher \n");
    }
    return;
  }

  /**
   * @brief Subscribe to a certain type of data in the system
   * @param taskToSubscribe Task Handle of the task that will receive
   *        and handle the data. (i.e. -> Subscribe<T>(this))
   */
  template <typename T>
  static void Subscribe(Task* taskToSubscribe) {
    if (subscriberListLock.Lock(SUBSCRIBER_LIST_MUTEX_TIMEOUT)) {
      Publisher<T>* publisher = getPublisher<T>();
      if (publisher != nullptr) {
        publisher->Subscribe(taskToSubscribe);
      } else {
        SOAR_ASSERT("Data Publisher not found \n");
      }
      subscriberListLock.Unlock();
      return;
    } else {
      SOAR_PRINT("Could Not Subscribe to Data Broker Publisher \n");
    }
    return;
  }

  /**
   * @brief Unsubscribe to a certain type of data in the system
   * @param taskToUnsubscribe Task Handle of the task that will stop
   *        receiving the data. (i.e. -> Unsubscribe<T>(this))
   */
  template <typename T>
  static void Unsubscribe(Task* taskToUnsubscribe) {
    if (subscriberListLock.Lock(SUBSCRIBER_LIST_MUTEX_TIMEOUT)) {
      Publisher<T>* publisher = getPublisher<T>();
      if (publisher != nullptr) {
        publisher->Unsubscribe(taskToUnsubscribe);
      } else {
        SOAR_ASSERT("Data Publisher not found \n");
      }
      subscriberListLock.Unlock();
      return;
    } else {
      SOAR_PRINT("Could Not Unsubscribe to Data Broker Publisher \n");
    }
    return;
  }

  /**
   * @brief This API can be used to offload the data from the databroker message
   *        into a new object in the receiving task
   * @param cm the Command object that contains the databroker message
   */
  template <typename T>
  static constexpr T ExtractData(const Command& cm) {
    if (cm.GetCommand() != DATA_BROKER_COMMAND) {
      SOAR_ASSERT("Not a Data Broker Command!\n");
    }

    Publisher<T>* publisher = getPublisher<T>();
    DataBrokerMessageTypes messageType = DataBroker::getMessageType(cm);

    if (messageType != publisher->GetPublisherMessageType()) {
      const std::string errorMessage = "Trying to unpack the wrong type of message. You are trying to use " +
                                       DataBrokerMessageType::ToString(publisher->GetPublisherMessageType()) +
                                       " instead of " + DataBrokerMessageType::ToString(messageType) + "\n\n";

      const char* messageCStr = errorMessage.c_str();
      SOAR_PRINT(messageCStr);
      SOAR_ASSERT(false, "");
    }

    // The data allocated by this command ptr will be freed when cm.Reset()]
    // is called. So we do not have to free this memory here
    T* dataPtr = reinterpret_cast<T*>(cm.GetDataPointer());

    T data{};

    std::memcpy(&data, dataPtr, sizeof(T));

    return data;
  }

  /**
   * @brief This API can be use to get the type of data broker message contained
   *        in the message.
   *        All the message types can be found in DataBrokerMessageTypes.hpp
   * @param cm the Command object that contains the databroker message
   */
  static DataBrokerMessageTypes getMessageType(const Command& cm) {
    return static_cast<DataBrokerMessageTypes>(cm.GetTaskCommand());
  }

 private:
  // Deleting the default constructor as this class is not
  // instanceable
  DataBroker() = delete;

  // Deleting the copy constructor to prevent copies
  DataBroker(const DataBroker& obj) = delete;

  // Deleting assignment operator to prevent assignment operations
  DataBroker& operator=(DataBroker const&) = delete;

  // Mutex to access the Subscriber List
  inline static Mutex subscriberListLock{};
  // Mutex lock wait time
  static constexpr uint16_t SUBSCRIBER_LIST_MUTEX_TIMEOUT = 1000;

  /**
   * @brief Checks if the 2 template types are the same
   */
  template <typename T, typename U>
  static constexpr bool matchType() {
    return std::is_same_v<T, U>;
  }

  /**
   * @brief Returns the correct Publisher object for a template type
   */
  template <typename T>
  static constexpr Publisher<T>* getPublisher(void) {
    if constexpr (matchType<T, IMUData>()) {
      return &IMU_Data_publisher;
    } else if constexpr (matchType<T, ThermocoupleData>()) {
      return &Thermocouple_Data_publisher;
    } else if constexpr (matchType<T, PressureData>()) {
    	return &Pressure_Data_publisher;
    }else {
      SOAR_ASSERT(false, "This publisher type does not exist, you must create it");
      return nullptr;
    }
  }

  // List of Publishers
  inline static Publisher<IMUData> IMU_Data_publisher{DataBrokerMessageTypes::IMU_DATA};
  inline static Publisher<PressureData> Pressure_Data_publisher{DataBrokerMessageTypes::PRESSURE_DATA};
  inline static Publisher<ThermocoupleData> Thermocouple_Data_publisher{DataBrokerMessageTypes::THERMOCOUPLE_DATA};
};
/************************************
 * FUNCTION DECLARATIONS
 ************************************/

#endif /* DATA_BROKER_HPP_ */
