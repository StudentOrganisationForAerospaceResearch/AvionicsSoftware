/**
 ******************************************************************************
 * File Name          : RocketSM.cpp
 * Description        : Primary rocket state machine
 ******************************************************************************
*/
#include "RocketSM.hpp"
#include "SystemDefines.hpp"
#include "FlashTask.hpp"
/* Rocket State Machine ------------------------------------------------------------------*/
/**
 * @brief Default constructor for Rocket SM, initializes all states
 */
RocketSM::RocketSM(RocketState startingState, bool enterStartingState)
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

    // Verify all states are initialized AND state IDs are consistent
    for (uint8_t i = 0; i < RS_NONE; i++) {
        SOAR_ASSERT(stateArray[i] != nullptr);
        SOAR_ASSERT(stateArray[i]->GetStateID() == i);
    }

    rs_currentState = stateArray[startingState];

    // If we need to run OnEnter for the starting state, do so
    if (enterStartingState) {
        rs_currentState->OnEnter();
    }

    SOAR_PRINT("Rocket State Machine Started in [ %s ] state\n", BaseRocketState::StateToString(rs_currentState->GetStateID()));
        
}

/**
 * @brief Handles state transitions
 * @param nextState The next state to transition to
 * @return The state after the transition
 */
RocketState RocketSM::TransitionState(RocketState nextState)
{
    // Check if we're already in the next state (TransitionState does not allow entry into the existing state)
    if (nextState == rs_currentState->GetStateID())
        return rs_currentState->GetStateID();

    // Check the next state is valid
    if (nextState >= RS_NONE)
        return rs_currentState->GetStateID();

    RocketState previousState = rs_currentState->GetStateID();

    // Exit the current state
    rs_currentState->OnExit();

    // Set the next state
    rs_currentState = stateArray[nextState];

    // Assert the next state is initalized
    SOAR_ASSERT(rs_currentState != nullptr, "rs_currentState is nullptr in TransitionState");

    // Enter the current state
    rs_currentState->OnEnter();

    SOAR_PRINT("ROCKET STATE TRANSITION [ %s ] --> [ %s ]\n", BaseRocketState::StateToString(previousState), BaseRocketState::StateToString(rs_currentState->GetStateID()));

    // Return the state after the transition
    return rs_currentState->GetStateID();
}

/**
 * @brief Handles current command
 * @param cm The command to handle
 */
void RocketSM::HandleCommand(Command& cm)
{
    SOAR_ASSERT(rs_currentState != nullptr, "Command received before state machine initialized");

    // Handle the command based on the current state
    RocketState nextRocketState = rs_currentState->HandleCommand(cm);

    // Run transition state - if the next state is the current state this does nothing
    if (nextRocketState != rs_currentState->GetStateID())
    {
        //send new state to FlashTask for storing
        Command cmd(DATA_COMMAND, (uint16_t)1);
        uint8_t state = rs_currentState->GetStateID();
        cmd.CopyDataToCommand(&state, 1);
        FlashTask::Inst().GetEventQueue()->Send(cmd);

        TransitionState(nextRocketState);
    }
}

/**
 * @brief Returns current state
 */
BaseRocketState* RocketSM::GetCurrentState()
{
    return rs_currentState;
}

/* Base State ------------------------------------------------------------------*/
///**
// * @brief General handler for actions that should be supported by all rocket state machines
// */
//RocketState BaseRocketState::HandleGeneralStateCommands(RocketControlCommands rcAction)
//{
//    switch (rcAction) {
//    case RSC_PAUSE_LOGGING:
//        //TODO: Send pause logging command
//        break;
//    case RSC_START_LOGGING:
//        //TODO: Send start logging command
//        break;
//    default:
//        break;
//    }
//
//    return GetStateID();
//}


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

    //uint8_t* crash = NULL;
    //*crash = 5;

    //if(&crash) {
        //return RS_ABORT;
    //}

    return rsStateID;
}

/**
 * @brief Handles control actions generally, can be used for derived states that allow full vent control
 * @return The rocket state to transition to or stay in. The current rocket state if no transition
 */
RocketState PreLaunch::HandleNonIgnitionCommands(RocketControlCommands rcAction, RocketState currentState)
{
    switch (rcAction) {
    case RSC_ANY_TO_ABORT:
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

    return currentState ;
}

/**
 * @brief HandleCommand for PreLaunch state
 * @return The rocket state to transition or stay in
 */
RocketState PreLaunch::HandleCommand(Command& cm)
{
    RocketState nextStateID = GetStateID();

    //// Handle general commands - these do not support state transitions
    //HandleGeneralStateCommands(cm.GetCommand());

    // Switch for the given command
    switch(cm.GetCommand()) {
    case CONTROL_ACTION: {
        switch (cm.GetTaskCommand()) {
        case RSC_GOTO_FILL:
            // Transition to fill state
            nextStateID = RS_FILL;
            break;
        default:
            // Handle as a general control action
            nextStateID = PreLaunch::HandleNonIgnitionCommands((RocketControlCommands)cm.GetTaskCommand(), GetStateID());
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
    for (uint8_t i = 0; i < 2; i++)
        arrArmConfirmFlags[i] = false;
}

/**
 * @brief Entry to Fill state
 * @return The state we're entering
 */
RocketState Fill::OnEnter()
{
    // Clear the arm flags
    for (uint8_t i = 0; i < 2; i++)
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
 * @brief HandleCommand for Fill state
 * @return The rocket state to transition or stay in
 */
RocketState Fill::HandleCommand(Command& cm)
{
    RocketState nextStateID = GetStateID();

    //// Handle general commands - these do not support state transitions
    //HandleGeneralStateCommands(cm.GetCommand());

    // Switch for the given command
    switch (cm.GetCommand()) {
    case CONTROL_ACTION: {
        switch (cm.GetTaskCommand()) {
        case RSC_ARM_CONFIRM_1:
            arrArmConfirmFlags[0] = true;
            break;
        case RSC_ARM_CONFIRM_2:
            arrArmConfirmFlags[1] = true;
            break;
        case RSC_GOTO_ARM:
            // Check if all arm confirmations have been received
            if (arrArmConfirmFlags[0] && arrArmConfirmFlags[1]) {
                // Transition to arm state
                nextStateID = RS_ARM;
            }
            break;
        case RSC_GOTO_PRELAUNCH:
            nextStateID = RS_PRELAUNCH;
            break;
        default:
            // Handle as a general control action
            nextStateID = PreLaunch::HandleNonIgnitionCommands((RocketControlCommands)cm.GetTaskCommand(), GetStateID());
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
 * @brief HandleCommand for Arm state
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
            //TODO: Transition to umbilical power - we should check to make sure umbilical power is available before doing so
            break;
        case RSC_POWER_TRANSITION_ONBOARD:
            //TODO: Transition to onboard power
            break;
        case RSC_GOTO_IGNITION:
            // Transition to ready for ignition state
            nextStateID = RS_IGNITION;
            break;
        case RSC_GOTO_FILL:
            nextStateID = RS_FILL;
            break;
        default:
            // If manual override is enabled, handle the command as a non ignition command
            nextStateID = PreLaunch::HandleNonIgnitionCommands((RocketControlCommands)cm.GetTaskCommand(), GetStateID());
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

/* Ignition State ------------------------------------------------------------------*/
/**
 * @brief Ignition state constructor
 */
Ignition::Ignition()
{
    rsStateID = RS_IGNITION;
}

/**
 * @brief Entry to Ignition state
 * @return The state we're entering
 */
RocketState Ignition::OnEnter()
{
    // We don't do anything upon entering ignition

    return rsStateID;
}

/**
 * @brief Exit from Ignition state
 * @return The state we're exiting
 */
RocketState Ignition::OnExit()
{


    return rsStateID;
}

/**
 * @brief HandleCommand for Ignition state
 * @return The rocket state to transition or stay in
 */
RocketState Ignition::HandleCommand(Command& cm)
{
    RocketState nextStateID = GetStateID();


    // Switch for the given command
    switch (cm.GetCommand()) {
    case CONTROL_ACTION: {
        switch (cm.GetTaskCommand()) {
        case RSC_IGNITION_TO_LAUNCH:
            nextStateID = RS_LAUNCH;
            break;
        case RSC_GOTO_ARM:
            // This is a transition directly to ARM (no checks required)
            nextStateID = RS_ARM;
            break;
        default:
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

/* Launch State ------------------------------------------------------------------*/
/**
 * @brief Launch state constructor
 */
Launch::Launch()
{
    rsStateID = RS_LAUNCH;
}

/**
 * @brief Entry to Launch state
 * @return The state we're entering
 */
RocketState Launch::OnEnter()
{
    //TODO: Disable Heartbeat Check
    //TODO: Ensure Vent & Drain CLOSED (**Should we not ensure vent & drain are closed in ignition? and or exit of ARM?)
    //TODO: Send command to OPEN the MEV
    //TODO: Immedietly transition to BURN .. ? (**Actually if we're transitioning right away why is this not both? ... Otherwise just queue up a command internally to GOTO BURN)

    return rsStateID;
}

/**
 * @brief Exit from Launch state
 * @return The state we're exiting
 */
RocketState Launch::OnExit()
{


    return rsStateID;
}

/**
 * @brief HandleCommand for Launch state
 * @return The rocket state to transition or stay in
 */
RocketState Launch::HandleCommand(Command& cm)
{
    RocketState nextStateID = GetStateID();

    // Switch for the given command
    switch (cm.GetCommand()) {
    case CONTROL_ACTION: {
        switch (cm.GetTaskCommand()) {
        case RSC_LAUNCH_TO_BURN:
            nextStateID = RS_BURN;
            break;
        default:
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

/* Burn State ------------------------------------------------------------------*/
/**
 * @brief Burn state constructor
 */
Burn::Burn()
{
    rsStateID = RS_BURN;
}

/**
 * @brief Entry to Burn state
 * @return The state we're entering
 */
RocketState Burn::OnEnter()
{
    //TODO: Validate Vent & Drain Closed
    //TODO: Start the coast transition timer (7 seconds - TBD based on sims)

    return rsStateID;
}

/**
 * @brief Exit from Burn state
 * @return The state we're exiting
 */
RocketState Burn::OnExit()
{


    return rsStateID;
}

/**
 * @brief HandleCommand for Burn state
 * @return The rocket state to transition or stay in
 */
RocketState Burn::HandleCommand(Command& cm)
{
    RocketState nextStateID = GetStateID();

    // Switch for the given command
    switch (cm.GetCommand()) {
    case CONTROL_ACTION: {
        switch (cm.GetTaskCommand()) {
        case RSC_BURN_TO_COAST:
            nextStateID = RS_COAST;
            break;
        default:
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

/* Coast State ------------------------------------------------------------------*/
/**
 * @brief Coast state constructor
 */
Coast::Coast()
{
    rsStateID = RS_COAST;
}

/**
 * @brief Entry to Coast state
 * @return The state we're entering
 */
RocketState Coast::OnEnter()
{
    //TODO: Start Descent Transition Timer (~25 seconds) : Should be well after apogee

    return rsStateID;
}

/**
 * @brief Exit from Coast state
 * @return The state we're exiting
 */
RocketState Coast::OnExit()
{

    return rsStateID;
}

/**
 * @brief HandleCommand for Coast state
 * @return The rocket state to transition or stay in
 */
RocketState Coast::HandleCommand(Command& cm)
{
    RocketState nextStateID = GetStateID();

    // Switch for the given command
    switch (cm.GetCommand()) {
    case CONTROL_ACTION: {
        switch (cm.GetTaskCommand()) {
        case RSC_COAST_TO_DESCENT:
            nextStateID = RS_DESCENT;
            break;
        default:
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

/* Descent State ------------------------------------------------------------------*/
/**
 * @brief Descent state constructor
 */
Descent::Descent()
{
    rsStateID = RS_DESCENT;
}

/**
 * @brief Entry to Descent state
 * @return The state we're entering
 */
RocketState Descent::OnEnter()
{
    //TODO: Start Recovery Transition Timer (~300 seconds) : Should be well into / after descent
    //TODO: Open Vent/Drain, Ensure MEV Closed

    return rsStateID;
}

/**
 * @brief Exit from Descent state
 * @return The state we're exiting
 */
RocketState Descent::OnExit()
{

    return rsStateID;
}

/**
 * @brief HandleCommand for Descent state
 * @return The rocket state to transition or stay in
 */
RocketState Descent::HandleCommand(Command& cm)
{
    RocketState nextStateID = GetStateID();

    // Switch for the given command
    switch (cm.GetCommand()) {
    case CONTROL_ACTION: {
        switch (cm.GetTaskCommand()) {
        case RSC_DESCENT_TO_RECOVERY:
            nextStateID = RS_RECOVERY;
            break;
        default:
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


/* Recovery State ------------------------------------------------------------------*/
/**
 * @brief Recovery state constructor
 */
Recovery::Recovery()
{
    rsStateID = RS_RECOVERY;
}

/**
 * @brief Entry to Recovery state
 * @return The state we're entering
 */
RocketState Recovery::OnEnter()
{
    //TODO: Open Vent & Drain (Maybe even periodic AUTO-VENT timers every 100 seconds to make sure they're open)
    //TODO: Send out GPS and GPIO Status (actually should be happening always anyway)
    //TODO: Decrease log rate to 1 Hz - StorageManager should automatically stop logging after it gets near full

    return rsStateID;
}

/**
 * @brief Exit from Recovery state
 * @return The state we're exiting
 */
RocketState Recovery::OnExit()
{

    return rsStateID;
}

/**
 * @brief HandleCommand for Recovery state
 * @return The rocket state to transition or stay in
 */
RocketState Recovery::HandleCommand(Command& cm)
{
    RocketState nextStateID = GetStateID();

    // Switch for the given command
    switch (cm.GetCommand()) {
    case CONTROL_ACTION: {
        switch (cm.GetTaskCommand()) {
        default:
            // Handle as a general control action
            nextStateID = PreLaunch::HandleNonIgnitionCommands((RocketControlCommands)cm.GetTaskCommand(), GetStateID());
            break;
        }
        break;
    }
    default:
        break;
    }

    // Make sure to reset the command, and return the next state
    cm.Reset();
    return nextStateID;
}

/* Abort State ------------------------------------------------------------------*/
/**
 * @brief Abort state constructor
 */
Abort::Abort()
{
    rsStateID = RS_ABORT;
}

/**
 * @brief Entry to Abort state
 * @return The state we're entering
 */
RocketState Abort::OnEnter()
{
    //TODO: Open Vent & Drain, MEV Closed

    return rsStateID;
}

/**
 * @brief Exit from Abort state
 * @return The state we're exiting
 */
RocketState Abort::OnExit()
{

    return rsStateID;
}

/**
 * @brief HandleCommand for Abort state
 * @return The rocket state to transition or stay in
 */
RocketState Abort::HandleCommand(Command& cm)
{
    RocketState nextStateID = GetStateID();

    // Switch for the given command
    switch (cm.GetCommand()) {
    case CONTROL_ACTION: {
        switch (cm.GetTaskCommand()) {
        case RSC_GOTO_PRELAUNCH:
            nextStateID = RS_PRELAUNCH;
            break;
        default:
            break;
        }
        break;
    }
    default:
        break;
    }

    // Make sure to reset the command, and return the next state
    cm.Reset();
    return nextStateID;
}

/**
 * @brief Returns a string for the state
 */
const char* BaseRocketState::StateToString(RocketState stateId)
{
    switch(stateId) {
    case RS_PRELAUNCH:
        return "Pre-launch";
    case RS_FILL:
        return "Fill";
    case RS_ARM:
        return "Arm";
    case RS_IGNITION:
        return "Ignition";
    case RS_LAUNCH:
        return "Launch";
    case RS_BURN:
        return "Burn";
    case RS_COAST:
        return "Coast";
    case RS_DESCENT:
        return "Descent";
    case RS_RECOVERY:
        return "Recovery";
    case RS_ABORT:
        return "Abort";
    case RS_NONE:
        return "None";
    default:
        return "WARNING: Invalid";
    }
}
