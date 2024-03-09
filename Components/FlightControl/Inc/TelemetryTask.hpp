/**
 ******************************************************************************
 * File Name          : TelemetryTask.hpp
 * Description        : Telemetry task controls the rate at which sensors or other
 *                      tasks send data to the ground station.
 ******************************************************************************
*/
#ifndef SOAR_TELEMETRYTASK_HPP_
#define SOAR_TELEMETRYTASK_HPP_
#include "Task.hpp"
#include "SystemDefines.hpp"

constexpr uint8_t NUM_FULL_LOGS_PER_GPS_LOG = 3 * 5; // N cycles of telemetry sends for each flash log

enum TELEMETRY_COMMANDS {
	TELEMETRY_DEBUG_PRINT_LOGMS,
	TELEMETRY_SET_LOG_RATE,
	TELEMETRY_SET_PROTOBUF_RATE,
	TELEMETRY_GET_LOG_RATE,
	TELEMETRY_GET_PROTOBUF_RATE,
	TELEMETRY_SET_BOTH_RATE
};

struct TelemetryRateConfig {
    uint32_t baro;
    uint32_t imu;
    uint32_t ptc;
    uint32_t gps;

    uint32_t fStatevDrainBattery;
};

struct BundledRates {
	TelemetryRateConfig flash;
	TelemetryRateConfig proto;
};


class TelemetryTask : public Task
{
public:
    static TelemetryTask& Inst() {
        static TelemetryTask inst;
        return inst;
    }

    void InitTask();

protected:
    static void RunTask(void* pvParams) { TelemetryTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();

    void Run(void* pvParams); // Main run code

    void HandleCommand(Command& cm);
    void RunLogSequence();

    void RequestSample();
    void RequestTransmit();
    void RequestLogToFlash();

    void SendVentDrainStatus();

    void SetFlashLogRate(TelemetryRateConfig newlograte);
    TelemetryRateConfig GetFlashLogRate();

    void SetProtobufLogRate(TelemetryRateConfig newprotobufrate);
    TelemetryRateConfig GetProtobufLogRate();


private:
    // Private Functions
    TelemetryTask();        // Private constructor
    TelemetryTask(const TelemetryTask&);                        // Prevent copy-construction
    TelemetryTask& operator=(const TelemetryTask&);            // Prevent assignment

    // Private Variables
    TelemetryRateConfig lograte;
    TelemetryRateConfig protobufrate;
    TelemetryRateConfig lastLogTicks;
    TelemetryRateConfig lastProtoTicks;

    uint16_t loggingDelayMs;

};

#endif    // SOAR_TELEMETRYTASK_HPP_
