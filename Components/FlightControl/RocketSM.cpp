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
    if (nextState == rs_currentState->GetStateName())
        return rs_currentState->GetStateName();

    // Exit the current state
    rs_currentState->OnExit();

    // Set the next state
    rs_currentState = stateArray[nextState];

    // Enter the current state
    rs_currentState->OnEnter();

    // Return the state after the transition
    return rs_currentState->GetStateName();
}


/* PreLaunch State ------------------------------------------------------------------*/
/**
 * @brief PreLaunch state constructor
 */
PreLaunch::PreLaunch()
{
    stateName = RS_PRELAUNCH;
}

/**
 * @brief Entry to PreLaunch state
 * @return The state we're entering
 */
RocketState PreLaunch::OnEnter()
{
    // We don't do anything upon entering prelaunch

    return stateName;
}

/**
 * @brief Exit from PreLaunch state
 * @return The state we're exiting
 */
RocketState PreLaunch::OnExit()
{
    // We don't do anything upon exiting prelaunch

    return stateName;
}


/* Some State ------------------------------------------------------------------*/

