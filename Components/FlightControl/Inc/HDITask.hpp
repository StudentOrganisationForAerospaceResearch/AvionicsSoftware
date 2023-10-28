/**
******************************************************************************
* File Name          : HDITask.hpp
* Description        : Primary flight task, default task for the system.
******************************************************************************
*/
#ifndef SOAR_HDI_HPP_
#define SOAR_HDI_HPP_
#include "RocketSM.hpp"
#include "SystemDefines.hpp"
#include "Task.hpp"

struct HDIConfig {
  uint8_t numBlinks;
  uint16_t delayMs;
};

enum HDITaskCommands : uint8_t {
  INVALID = 0,

  MUTE = 0x11,
  UNMUTE = 0x22,
};

class HDITask : public Task {
 public:
  static HDITask& Inst() {
    static HDITask inst;
    return inst;
  }
  void InitTask();
  RocketState currentHDIState();

 protected:
  static void RunTask(void* pvParams) {
    HDITask::Inst().Run(pvParams);
  }  // Static Task Interface, passes control to the instance Run();

  void Run(void* pvParams);  // Main run code
  void BuzzBlinkSequence(HDIConfig blinkSequence);
  void HandleRequestCommand(uint16_t taskCommand);
  void HandleCommand(Command& cm);

 private:
  // Private Functions
  HDITask();                           // Private constructor
  HDITask(const HDITask&);             // Prevent copy-construction
  HDITask& operator=(const HDITask&);  // Prevent assignment
  HDIConfig currentConfig;

  bool buzzerMuted_;
};

#endif  // SOAR_FLIGHTTASK_HPP_
