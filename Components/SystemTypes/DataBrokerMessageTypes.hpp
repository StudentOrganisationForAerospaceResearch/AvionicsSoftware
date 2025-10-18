/**
 ********************************************************************************
 * @file    DataBrokerMessageTypes.hpp
 * @author  Shivam Desai
 * @date    Nov 23, 2024
 * @brief
 ********************************************************************************
 */

#ifndef DATA_BROKER_MESSAGE_TYPES_HPP_
#define DATA_BROKER_MESSAGE_TYPES_HPP_

/************************************
 * INCLUDES
 ************************************/
#include <stdint.h>
#include <string>

/************************************
 * MACROS AND DEFINES
 ************************************/

/************************************
 * TYPEDEFS
 ************************************/
enum class DataBrokerMessageTypes : uint8_t {
  INVALID = 0,
  GYROSCOPE_DATA,
  THERMOCOUPLE_DATA,
  PRESSURE_DATA,
  ACCELEROMETER_DATA
};

namespace DataBrokerMessageType {
/************************************
 * CLASS DEFINITIONS
 ************************************/

/************************************
 * FUNCTION DECLARATIONS
 ************************************/
std::string ToString(DataBrokerMessageTypes messageType);

inline std::string ToString(DataBrokerMessageTypes messageType) {
  switch (messageType) {
    case DataBrokerMessageTypes::GYROSCOPE_DATA: {
      std::string type{"GYROSCOPE_DATA"};
      return type;
    }

    case DataBrokerMessageTypes::THERMOCOUPLE_DATA: {
      std::string type{"THERMOCOUPLE_DATA"};
      return type;
    }

    case DataBrokerMessageTypes::ACCELEROMETER_DATA: {
    	std::string type{"ACCELEROMETER_DATA"};
    	return type;
    }

    default: {
      std::string type{"INVALID"};
      return type;
    }
  }
}

}  // namespace DataBrokerMessageType

#endif /* DATA_BROKER_MESSAGE_TYPES_HPP_ */
