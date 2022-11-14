/**
 ******************************************************************************
 * File Name          : RocketSM.hpp
 * Description        : Rocket state machine, handles all rocket state transitions.
 ******************************************************************************
*/
#ifndef SOAR_AVIONICS_ROCKET_SM
#define SOAR_AVIONICS_ROCKET_SM

#include "Command.hpp"

enum RocketState
{
    RS_PRELAUNCH = 0,   // Idle state, waiting for command to proceeding sequences
    RS_PREFILL,     // Pressure testing of N2 (?) and LOX
    RS_FILL,        // Control of solenoid valves
    RS_ARM,         // We don't allow fill etc. 1-2 minutes before launch
    RS_IGNITION,    // Ignition of the ignitors
    RS_LAUNCH,      // Launch triggered by confirmation of ignition (from ignitor) is nominal
    RS_BURN,        // Main burn (vents closed MEV open) - 5-6 seconds (TBD)
    RS_COAST,       // Coasting (MEV closed, vents closed) - 30 seconds (TBD)
    RS_POSTAPOGEE,  // Vents open (well into the descent)
    RS_ABORT,       // Abort sequence, vents open, MEV closed, ignitors off
    RS_NONE         // Invalid state, must be last
};

/**
 * @brief Base class for Rocket State Machine
 */
class BaseRocketState
{
public:
    virtual void HandleCommand(Command& cm) = 0; //Handle a command based on the current state
    virtual RocketState OnEnter() = 0;  //Returns the state we're entering
    virtual RocketState OnExit() = 0;   //Returns the state we're exiting

    RocketState GetStateName() { return stateName; }
protected:
    RocketState stateName = RS_NONE;    //The name of the state we're in

};

/**
 * @brief Rocket State Machine
 */
class RocketControl
{
public:
    RocketControl();

    void HandleCommand(Command& cm);

protected:
    RocketState TransitionState(RocketState nextState);

    // Variables
    BaseRocketState* stateArray[RS_NONE-1];
    BaseRocketState* rs_currentState;
};

/**
 * @brief PreLaunch state, waiting for commands to proceeding sequences
 */
class PreLaunch : public BaseRocketState
{
public:
    PreLaunch();

    void HandleCommand(Command& cm);
    RocketState OnEnter();
    RocketState OnExit();
};







#endif // SOAR_AVIONICS_ROCKET_SM