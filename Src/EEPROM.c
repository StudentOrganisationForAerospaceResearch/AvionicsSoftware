/**
  ******************************************************************************
  * File Name          : EEPROM.c
  * Description        : This file contains utilities to interact with and
  *                      subsequently use the EEPROM data storage chip
  *                      introduced on the Andromeda V3.2 flight board.
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/

#include <EEPROM.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "Data.h"

/* Macros --------------------------------------------------------------------*/

/* Constants -----------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

/* Structs -------------------------------------------------------------------*/

/* Prototypes ----------------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/

/**
 * This function is to be used as a thread task that manages and performs any
 * interaction with the EEPROM itself, to save other threads from any possible
 * performance delays.
 *
 * @param   arg Unused.
 */
void EEPROMTask(void const* arg)
{
    for (;;)
    {
    	;
    }
}
