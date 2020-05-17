/**
  ******************************************************************************
  * File Name          : ValveControl.h
  ******************************************************************************
*/

#pragma once

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Macros --------------------------------------------------------------------*/
#define MAX_DURATION_VENT_VALVE_OPEN (8000)        // Required in AbortPhase.c
#define REQUIRED_DURATION_VENT_VALVE_CLOSED (4000) // Required in AbortPhase.c

/* Structs -------------------------------------------------------------------*/

/* Externs -------------------------------------------------------------------*/

/* Prototypes ----------------------------------------------------------------*/
void openVentValve();
void closeVentValve();
void openInjectionValve();
void closeInjectionValve();
void openLowerVentValve();
void closeLowerVentValve();
