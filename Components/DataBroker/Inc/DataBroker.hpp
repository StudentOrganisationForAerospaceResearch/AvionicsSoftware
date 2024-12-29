/**
 ********************************************************************************
 * @file    DataBroker.hpp
 * @author  shivam
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
      }
      else {
        SOAR_ASSERT("Data Publisher not found \n");
      }
      subscriberListLock.Unlock();
      return;
    }
    else {
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
      }
      else {
        SOAR_ASSERT("Data Publisher not found \n");
      }
      subscriberListLock.Unlock();
      return;
    }
    else {
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
      }
      else {
        SOAR_ASSERT("Data Publisher not found \n");
      }
      subscriberListLock.Unlock();
      return;
    }
    else {
      SOAR_PRINT("Could Not Unsubscribe to Data Broker Publisher \n");
    }
    return;
  }

  template <typename T>
  static constexpr T ExtractData(const Command &cm) {
    if (cm.GetCommand() != DATA_BROKER_COMMAND) {
      SOAR_ASSERT("Not a Data Broker Command!\n");
    }

    // The data allocated by this command ptr will be freed when cm.Reset()]
    // is called. So we do not have to free this memory here
    T* dataPtr = reinterpret_cast<T*>(cm.GetDataPointer());

    T data{};

    std::memcpy(&data, dataPtr, sizeof(T));

    return data;
  }

  static DataBrokerMessageTypes getMessageType(const Command &cm) {
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

  // matcher - match template type with publisher type
  template <typename T, typename U>
  static constexpr bool matchType() {
    return std::is_same_v<T, U>;
  }

  // get data publisher
  template <typename T>
  static constexpr auto getPublisher(void) {
    if constexpr (matchType<T, IMUData>()) {
      return &IMU_Data_publisher;
    } else if constexpr (matchType<T, ThermocoupleData>()) {
      return &Thermocouple_Data_publisher;
    } else {
      SOAR_ASSERT(false, "This publisher type does not exist, you must create it");
    }
  }

  // list of publishers
  inline static Publisher<IMUData> IMU_Data_publisher {DataBrokerMessageTypes::IMU_DATA};
  inline static Publisher<ThermocoupleData> Thermocouple_Data_publisher {DataBrokerMessageTypes::THERMOCOUPLE_DATA};

};
/************************************
 * FUNCTION DECLARATIONS
 ************************************/

#endif /* DATA_BROKER_HPP_ */
