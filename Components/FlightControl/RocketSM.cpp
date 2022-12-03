/**
 ******************************************************************************
 * File Name          : RocketSM.cpp
 * Description        : Primary rocket state machine
 ******************************************************************************
*/
#include "RocketSM.hpp"
#include "SystemDefines.hpp"
/* Rocket Control ------------------------------------------------------------------*/
/**
 * @brief Default constructor for Rocket SM, initializes all states
 */
RocketControl::RocketControl()
{
    // Setup the internal array of states. Setup in order of enum.
    stateArray[RS_PRELAUNCH] = new PreLaunch();
    stateArray[RS_FILL] = new Fill();
    stateArray[RS_ARM] = new Arm();
    stateArray[RS_IGNITION] = new Ignition();
    stateArray[RS_LAUNCH] = new Launch();
    stateArray[RS_BURN] = new Burn();
    stateArray[RS_COAST] = new Coast();
    stateArray[RS_DESCENT] = new Descent();
    stateArray[RS_RECOVERY] = new Recovery();
    stateArray[RS_ABORT] = new Abort();

    // Verify all states are initialized
    for (uint8_t i = 0; i < RS_NONE-1; i++)
        SOAR_ASSERT(stateArray[i] != nullptr);
}

/**
 * @brief Handles state transitions
 * @param nextState The next state to transition to
 * @return The state after the transition
 */
RocketState RocketControl::TransitionState(RocketState nextState)
{
    // Check if we're already in the next state (TransitionState does not allow entry into the existing state)
    if (nextState == rs_currentState->GetStateID())
        return rs_currentState->GetStateID();

    // Check the next state is valid
    if (nextState >= RS_NONE)
        return rs_currentState->GetStateID();

    // Exit the current state
    rs_currentState->OnExit();

    // Set the next state
    rs_currentState = stateArray[nextState];

    // Assert the next state is initalized
    SOAR_ASSERT(rs_currentState != nullptr, "rs_currentState is nullptr in TransitionState");

    // Enter the current state
    rs_currentState->OnEnter();

    // Return the state after the transition
    return rs_currentState->GetStateID();
}

/**
 * @brief Handles current command
 * @param cm The command to handle
 */
void RocketControl::HandleCommand(Command& cm)
{
    SOAR_ASSERT(rs_currentState != nullptr, "Command received before state machine initialized");

    // Handle the command based on the current state
    rs_currentState->HandleCommand(cm);
}

/* Base State ------------------------------------------------------------------*/
/**
 * @brief General handler for actions that should be supported by all rocket state machines
 */
RocketState BaseRocketState::HandleGeneralStateCommands(RocketControlCommands rcAction)
{
    switch (rcAction) {
    case RSC_PAUSE_LOGGING:
        //TODO: Send pause logging command
        break;
    case RSC_START_LOGGING:
        //TODO: Send start logging command
        break;
    default:
        break;
    }

    return GetStateID();
}


/* PreLaunch State ------------------------------------------------------------------*/
/**
 * @brief PreLaunch state constructor
 */
PreLaunch::PreLaunch()
{
    rsStateID = RS_PRELAUNCH;
}

/**
 * @brief Entry to PreLaunch state
 * @return The state we're entering
 */
RocketState PreLaunch::OnEnter()
{
    // We don't do anything upon entering prelaunch

    return rsStateID;
}

/**
 * @brief Exit from PreLaunch state
 * @return The state we're exiting
 */
RocketState PreLaunch::OnExit()
{
    // We don't do anything upon exiting prelaunch

    return rsStateID;
}

/**
 * @brief Handles control actions generally, can be used for derived states that allow full vent control
 * @return The rocket state to transition to or stay in. The current rocket state if no transition
 */
RocketState PreLaunch::HandleNonIgnitionCommands(RocketControlCommands rcAction)
{
    switch (rcAction) {
    case RSC_ABORT:
        // Transition to abort state
        return RS_ABORT;
    case RSC_OPEN_VENT:
        //TODO: Open the vent valve
        break;
    case RSC_CLOSE_VENT:
        //TODO: Close the vent valve
        break;
    case RSC_OPEN_DRAIN:
        //TODO: Close the drain
        break;
    case RSC_CLOSE_DRAIN:
        //TODO: Open the drain
        break;
    case RSC_MEV_CLOSE:
        //TODO: Close the MEV
        break;
    default:
    	break;
    }

    return GetStateID();
}

/**
 * @brief HandleCommand for PreLaunch state
 * @return The rocket state to transition or stay in
 */
RocketState PreLaunch::HandleCommand(Command& cm)
{
    RocketState nextStateID = GetStateID();

    // Handle general commands - these do not support state transitions
    HandleGeneralStateCommands(cm.GetCommand());

    // Switch for the given command
    switch(cm.GetCommand()) {
    case CONTROL_ACTION: {
        switch (cm.GetTaskCommand()) {
        case RSC_BEGIN_FILL:
            // Transition to fill state
            nextStateID = RS_FILL;
            break;
        default:
            // Handle as a general control action
            nextStateID = HandleNonIgnitionCommands((RocketControlCommands)cm.GetTaskCommand());
            break;
        }
        break;
    }
    default:
        // Do nothing
        break;
    }

    // Make sure to reset the command, and return the next state
    cm.Reset();
    return nextStateID;
}



/* Fill State ------------------------------------------------------------------*/
/**
 * @brief Fill state constructor
 */
Fill::Fill()
{
    rsStateID = RS_FILL;

    // Clear the arm flags
    for (uint8_t i = 0; i < 3; i++)
        arrArmConfirmFlags[i] = false;
}

/**
 * @brief Entry to Fill state
 * @return The state we're entering
 */
RocketState Fill::OnEnter()
{
    // Clear the arm flags
    for (uint8_t i = 0; i < 3; i++)
        arrArmConfirmFlags[i] = false;

    // TODO: Consider automatically beginning fill sequence (since we've already explicitly entered the fill state)

    return rsStateID;
}

/**
 * @brief Exit from Fill state
 * @return The state we're exiting
 */
RocketState Fill::OnExit()
{
    // Clear the arm flags
    for(uint8_t i = 0; i < 3; i++)
        arrArmConfirmFlags[i] = false;

    return rsStateID;
}

/**
 * @brief HandleCommand for PreLaunch state
 * @return The rocket state to transition or stay in
 */
RocketState Fill::HandleCommand(Command& cm)
{
    RocketState nextStateID = GetStateID();

    // Handle general commands - these do not support state transitions
    HandleGeneralStateCommands(cm.GetCommand());

    // Switch for the given command
    switch (cm.GetCommand()) {
    case CONTROL_ACTION: {
        switch (cm.GetTaskCommand()) {
        case RSC_FILLTEST:
            // TODO: Initiate fill test sequence
            break;
        case RSC_ARM_CONFIRM_1:
            arrArmConfirmFlags[0] = true;
            break;
        case RSC_ARM_CONFIRM_2:
            arrArmConfirmFlags[1] = true;
            break;
        case RSC_ARM_CONFIRM_3:
            arrArmConfirmFlags[2] = true;
            break;
        case RSC_ARM_ACTION:
            // Check if all arm confirmations have been received
            if (arrArmConfirmFlags[0] && arrArmConfirmFlags[1] && arrArmConfirmFlags[2]) {
                // Transition to arm state
                nextStateID = RS_ARM;
            }
            break;
        default:
            // Handle as a general control action
            nextStateID = HandleNonIgnitionCommands((RocketControlCommands)cm.GetTaskCommand());
            break;
        }
        break;
    }
    default:
        // Do nothing
        break;
    }

    // Make sure to reset the command, and return the next state
    cm.Reset();
    return nextStateID;
}

/* Arm State ------------------------------------------------------------------*/
/**
 * @brief Arm state constructor
 */
Arm::Arm()
{
    rsStateID = RS_ARM;
}

/**
 * @brief Entry to Arm state
 * @return The state we're entering
 */
RocketState Arm::OnEnter()
{
    // We don't do anything upon entering arm
    // TODO: Consider automatically beginning arm sequence (since we've already explicitly entered the arm state)

    return rsStateID;
}

/**
 * @brief Exit from Arm state
 * @return The state we're exiting
 */
RocketState Arm::OnExit()
{


    return rsStateID;
}

/**
 * @brief HandleCommand for PreLaunch state
 * @return The rocket state to transition or stay in
 */
RocketState Arm::HandleCommand(Command& cm)
{
    RocketState nextStateID = GetStateID();


    // Switch for the given command
    switch (cm.GetCommand()) {
    case CONTROL_ACTION: {
        switch (cm.GetTaskCommand()) {
        case RSC_POWER_TRANSITION_EXTERNAL:
            // TODO: Transition to umbilical power - we should check to make sure umbilical power is available before doing so
            break;
        case RSC_POWER_TRANSITION_ONBOARD:
            //TODO: Transition to onboard power
            break;
        case RSC_FILLARM_DISCONNECT:
            //TODO: Fill arm disconnect sequence
            break;
        case RSC_INSULATION_REMOVE:
            //TODO: Remove insulation
            break;
        case RSC_INSULATION_APPLY:
            //TODO: Apply insulation
            break;
        case RSC_READY_FOR_IGNITION:
            // Transition to ready for ignition state
            nextStateID = RS_IGNITION;
            break;
        case RSC_MANUAL_OVERRIDE_ENABLE:
            isManualOverrideEnabled = true;
            break;
        default:
            // If manual override is enabled, handle the command as a non ignition command
            if (isManualOverrideEnabled) {
                nextStateID = HandleNonIgnitionCommands((RocketControlCommands)cm.GetTaskCommand());
                isManualOverrideEnabled = false;
            }
            break;
        }
        break;
    }
    default:
        // Do nothing
        break;
    }

    // Make sure to reset the command, and return the next state
    cm.Reset();
    return nextStateID;
}

