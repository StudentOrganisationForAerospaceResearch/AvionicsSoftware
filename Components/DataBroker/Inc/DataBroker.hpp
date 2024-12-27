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

	// list of publishers
	static Publisher<IMUData> IMU_Data_publisher;
	static Publisher<ThermocoupleData> Thermocouple_Data_publisher;

  // publish system message
  template <typename T>
  static void PublishData(T* dataToPublish) {
  	auto publisher = getPublisher<T>();
    if (publisher != nullptr) {
    	publisher->Publish(dataToPublish);
    }
    else {
    	SOAR_PRINT("Data Publisher ERROR \n");
    }
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

};
/************************************
 * FUNCTION DECLARATIONS
 ************************************/

#endif /* DATA_BROKER_HPP_ */
