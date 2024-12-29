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
#include <type_traits>

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
   * 				NOTE: You must ensure that there is a publisher for that type
   */
  template <typename T>
  static void PublishData(T* dataToPublish) {
  	Publisher<T>* publisher = getPublisher<T>();
//  	SOAR_PRINT("\nADDRESS OF PUBLISHER :: %d \n", publisher);
    if (publisher != nullptr) {
    	publisher->Publish(dataToPublish);
    }
    else {
    	SOAR_ASSERT("Data Publisher not found \n");
    }
  }

  /**
   * @brief Subscribe to a certain type of data in the system
   * @param taskToSubscribe Task Handle of the task that will receive
   *        and handle the data. (i.e. -> Subscribe(this))
   */
  template <typename T>
  static void Subscribe(Task* taskToSubscribe) {
  	Publisher<T>* publisher = getPublisher<T>();
//  	SOAR_PRINT("\nADDRESS OF PUBLISHER SUBSCRIBED TOO :: %d\n", publisher);
		if (publisher != nullptr) {
			publisher->Subscribe(taskToSubscribe);
		}
		else {
			SOAR_ASSERT("Data Publisher not found \n");
		}
  }

  static constexpr DataBrokerMessageTypes getDataBrokerMessageType(uint16_t messageType) {
  	return static_cast<DataBrokerMessageTypes>(messageType);
  }

 private:
  // Deleting the default constructor as this class is not
  // instanceable
  DataBroker() = delete;

	// Deleting the copy constructor to prevent copies
	DataBroker(const DataBroker& obj) = delete;

	// Deleting assignment operator to prevent assignment operations
	DataBroker& operator=(DataBroker const&) = delete;

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
