/**
  ******************************************************************************
  * File Name          : ReadBarometer.h
  ******************************************************************************
*/

#pragma once

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Macros --------------------------------------------------------------------*/

/* Structs -------------------------------------------------------------------*/

/* Externs -------------------------------------------------------------------*/
extern SPI_HandleTypeDef hspi2;

/* Prototypes ----------------------------------------------------------------*/
void readBarometerTask(void const* arg);
