/**
 ********************************************************************************
 * @file    DataBrokerMessageTypes.hpp
 * @author  shivam
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

/************************************
 * MACROS AND DEFINES
 ************************************/

/************************************
 * TYPEDEFS
 ************************************/
enum class DataBrokerMessageTypes : uint8_t {
  INVALID = 0,
  IMU_DATA,
  THERMOCOUPLE_DATA,
};

/************************************
 * CLASS DEFINITIONS
 ************************************/

/************************************
 * FUNCTION DECLARATIONS
 ************************************/

#endif /* DATA_BROKER_MESSAGE_TYPES_HPP_ */
