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
    //-- GROUND --
    // Manual venting allowed at all times
    RS_PRELAUNCH = 0,   // Idle state, waiting for command to proceeding sequences
    RS_FILL,        // N2 Prefill/Purge/Leak-check/Load-cell Tare check sub-sequences, full control of valves (except MEV) allowed
    RS_ARM,         // We don't allow fill etc. 1-2 minutes before launch : Cannot fill rocket with N2 etc. unless you return to FILL
                    // Power Transition, Fill-Arm Disconnect Sub-sequences (you should be able to revert the power transition)

    //-- IGNITION -- Manual venting NOT ALLOWED
    RS_IGNITION,    // Ignition of the ignitors
    RS_LAUNCH,      // Launch triggered by confirmation of ignition (from ignitor) is nominal : MEV Open Sequence

    //-- BURN --
    // Vents should stay closed, manual venting NOT ALLOWED
    // !vents open is definitely not ideal for abort! Best to keep it closed with manual override if we fail here
    // (can we maybe have the code change the true default state by overwriting EEPROM?)
    // Ideally we don't want to EVER exceed 7 seconds of burn time (we should store this time at the very least - split into 1 sub-stage for each second if necessary).
    // For timing we want ~1/10th of a second or better.
    RS_BURN,        // Main burn (vents closed MEV open) - 5-6 seconds (TBD) :
                    

    //-- COAST --
    // Manual venting NOT ALLOWED
    RS_COAST,       // Coasting (MEV closed, vents closed) - 30 seconds (TBD) ^ Vents closed applies here too, in part. Includes APOGEE

    //-- DESCENT / POSTAPOGEE --
    // Automatic Venting AND Vent Control ALLOWED
    RS_DESCENT,  // Vents open (well into the descent)
    RS_POSTLAUNCH,  // Vents open, MEV closed, transmit all data over radio and accept vent commands

    //-- RECOVERY / TECHNICAL --
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