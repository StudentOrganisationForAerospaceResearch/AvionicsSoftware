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
    RS_RECOVERY,  // Vents open, MEV closed, transmit all data over radio and accept vent commands
                  // Supports general commands (e.g. venting) and logs/transmits slowly (maybe stop logging after close to full memory?)

    //-- RECOVERY / TECHNICAL --
    RS_ABORT,       // Abort sequence, vents open, MEV closed, ignitors off
    RS_NONE         // Invalid state, must be last
};

/**
 * @brief External Rocket Control Commands, all fall under GLOBAL_COMMAND -> CONTROL_ACTION umbrella
 *
 *        State specific commands, must be all in-order to avoid duplicate command IDs
 */
enum RocketControlCommands
{
    RSC_FIRST_INVALID = 0,

    //-- PRE-IGNITION and RECOVERY --
    RSC_ANY_TO_ABORT,       // Transition to ABORT state - available from all states except for IGNITION/LAUNCH/BURN
    RSC_OPEN_VENT,   // Open the vent valve
    RSC_CLOSE_VENT,  // Close the vent valve
    RSC_OPEN_DRAIN,  // Open the drain valve
    RSC_CLOSE_DRAIN, // Close the drain valve
    RSC_MEV_CLOSE,   // Forces MEV to close - ONLY supported in states where it is safe to close the MEV

    //-- PRELAUNCH --
    RSC_GOTO_FILL, // Transition to the FILL state

    //-- FILL --
    RSC_ARM_CONFIRM_1,   // Enable first ARM confirmation flag
    RSC_ARM_CONFIRM_2,   // Enable second ARM confirmation flag
    RSC_GOTO_ARM,      // Transition to the ARM state (not allowed without the confirm flags set)
    RSC_GOTO_PRELAUNCH, // Transition to the PRELAUNCH state from FILL

    //-- ARM/IGNITION/LAUNCH/BURN --

    //-- ARM --
    RSC_POWER_TRANSITION_ONBOARD,      // Change power source to onboard
    RSC_POWER_TRANSITION_EXTERNAL,     // Change power source to external power
    //RSC_GOTO_FILL, // Transition back
    RSC_GOTO_IGNITION, // Ready for ignition sequence - Transition to IGNITION state

    //-- IGNITION --
    RSC_IGNITION_TO_LAUNCH,   // Confirm igniter actuation - Transition to LAUNCH state (MEV OPEN)
    //RSC_GOTO_ARM,      // Non-confirm igniter actuation - Transition back to ARM state   

    //-- LAUNCH --
    // * These flight sequence commands can be replaced with direct calls to transition state IF possible
    RSC_LAUNCH_TO_BURN, // Internal command, should not be triggered externally
    //-- BURN --
    RSC_BURN_TO_COAST, // Internal command, should not be triggered externally
    //-- COAST --
    RSC_COAST_TO_DESCENT, // Internal command, should not be triggered externally
    //-- DESCENT --
    RSC_DESCENT_TO_RECOVERY, // Internal command, should not be triggered externally

    //-- ABORT --
    //RSC_GOTO_PRELAUNCH, // Confirm transition back into prelaunch state

    //-- GENERAL --
    RSC_MANUAL_IGNITION_CONFIRMED,
    RSC_IR_IGNITION_CONFIRMED,

    //-- TIMER STATE TRANSITIONS --
    RSC_IGINITION_CMD,  // UNUSED, WAS CREATED FOR TESTING PURPOSES

    //-- TECHNICAL --
    RSC_NONE   // Invalid command, must be last
};

/**
 * @brief Base class for Rocket State Machine
 */
class BaseRocketState
{
public:
    virtual RocketState HandleCommand(Command& cm) = 0; //Handle a command based on the current state
    virtual RocketState OnEnter() = 0;  //Returns the state we're entering
    virtual RocketState OnExit() = 0;   //Returns the state we're exiting

    virtual RocketState GetStateID() { return rsStateID; }

    static const char* StateToString(RocketState stateId);

    //RocketState HandleGeneralStateCommands(RocketControlCommands rcAction);
protected:
    RocketState rsStateID = RS_NONE;    //The name of the state we're in

};

/**
 * @brief Rocket State Machine
 */
class RocketSM
{
public:
    RocketSM(RocketState startingState, bool enterStartingState);

    void HandleCommand(Command& cm);

protected:
    RocketState TransitionState(RocketState nextState);

    // Variables
    BaseRocketState* stateArray[RS_NONE];
    BaseRocketState* rs_currentState;
};

/**
 * @brief PreLaunch state, waiting for commands to proceeding sequences
 */
class PreLaunch : public BaseRocketState
{
public:
    PreLaunch();

    // Base class
    RocketState HandleCommand(Command& cm) override;
    RocketState OnEnter() override;
    RocketState OnExit() override;
    
    // Non-ignition
    static RocketState HandleNonIgnitionCommands(RocketControlCommands rcAction, RocketState currentState);
};

/**
 * @brief Fill state, N2 Prefill/Purge/Leak-check/Load-cell Tare check sub-sequences, full control of valves (except MEV) allowed
 */
class Fill : public BaseRocketState
{
public:
    Fill();

    RocketState HandleCommand(Command& cm) override;
    RocketState OnEnter() override;
    RocketState OnExit() override;

private:
    bool arrArmConfirmFlags[2]; // we have three arm confirm flags
};

/**
 * @brief Arm state, we don't allow fill etc. 1-2 minutes before launch : Cannot fill rocket with N2 etc. unless you return to FILL
 */
class Arm : public BaseRocketState
{
public:
    Arm();

    RocketState HandleCommand(Command& cm) override;
    RocketState OnEnter() override;
    RocketState OnExit() override;

private:
};

/**
 * @brief Ignition state, ignition of the ignitors
 */
class Ignition : public BaseRocketState
{
public:
    Ignition();

    RocketState HandleCommand(Command& cm) override;
    RocketState OnEnter() override;
    RocketState OnExit() override;
};

/**
 * @brief Launch state, launch triggered by confirmation of ignition (from ignitor) is nominal : MEV Open Sequence
 */
class Launch : public BaseRocketState
{
public:
    Launch();

    RocketState HandleCommand(Command& cm) override;
    RocketState OnEnter() override;
    RocketState OnExit() override;
};

/**
 * @brief Burn state, main burn (vents closed MEV open) - 5-6 seconds (TBD) :
 */
class Burn : public BaseRocketState
{
public:
    Burn();

    RocketState HandleCommand(Command& cm) override;
    RocketState OnEnter() override;
    RocketState OnExit() override;
};

/**
 * @brief Coast state, coasting (MEV closed, vents closed) - 30 seconds (TBD) ^ Vents closed applies here too, in part. Includes APOGEE
 */
class Coast : public BaseRocketState
{
public:
    Coast();

    RocketState HandleCommand(Command& cm) override;
    RocketState OnEnter() override;
    RocketState OnExit() override;
};

/**
 * @brief Descent state, vents open (well into the descent)
 */
class Descent : public BaseRocketState
{
public:
    Descent();

    RocketState HandleCommand(Command& cm) override;
    RocketState OnEnter() override;
    RocketState OnExit() override;
};

/**
 * @brief PostLaunch state, vents open, MEV closed, transmit all data over radio and accept vent commands
 */
class Recovery : public BaseRocketState
{
public:
    Recovery();

    RocketState HandleCommand(Command& cm) override;
    RocketState OnEnter() override;
    RocketState OnExit() override;
};

/**
 * @brief Abort state, abort sequence, vents open, MEV closed, ignitors off
 */
class Abort : public BaseRocketState
{
public:
    Abort();

    RocketState HandleCommand(Command& cm) override;
    RocketState OnEnter() override;
    RocketState OnExit() override;
};


#endif // SOAR_AVIONICS_ROCKET_SM
