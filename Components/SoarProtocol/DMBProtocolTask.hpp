/**
 ******************************************************************************
 * File Name          : DMBProtocolTask.hpp
 * Description        : Protocol task, specific to DMB
 ******************************************************************************
*/
#ifndef SOAR_DMBPROTOCOL_HPP_
#define SOAR_DMBPROTOCOL_HPP_
#include "ProtocolTask.hpp"
#include "SystemDefines.hpp"
#include "Task.hpp"
#include "UARTTask.hpp"

/* Enums ------------------------------------------------------------------*/

/* Class ------------------------------------------------------------------*/
class DMBProtocolTask : public ProtocolTask {
 public:
  static DMBProtocolTask& Inst() {
    static DMBProtocolTask inst;
    return inst;
  }

  void InitTask();

  static void SendProtobufMessage(
      EmbeddedProto::WriteBufferFixedSize<DEFAULT_PROTOCOL_WRITE_BUFFER_SIZE>&
          writeBuffer,
      Proto::MessageID msgId) {
    Inst().ProtocolTask::SendProtobufMessage(writeBuffer, msgId);
  }

 protected:
  static void RunTask(void* pvParams) {
    DMBProtocolTask::Inst().Run(pvParams);
  }  // Static Task Interface, passes control to the instance Run();

  // These handlers will receive a buffer and size corresponding to a decoded message
  void HandleProtobufCommandMessage(
      EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES>&
          readBuffer);
  void HandleProtobufControlMesssage(
      EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES>&
          readBuffer);
  void HandleProtobufTelemetryMessage(
      EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES>&
          readBuffer);

  // Member variables

 private:
  DMBProtocolTask();                        // Private constructor
  DMBProtocolTask(const DMBProtocolTask&);  // Prevent copy-construction
  DMBProtocolTask& operator=(const DMBProtocolTask&);  // Prevent assignment
};

#endif  // SOAR_DMBPROTOCOL_HPP_
