#ifndef __STATUSLED_H__
#define __STATUSLED_H__

/*!
 * \file StatusLED.h
 * \author Lukas Otava
 * \date 2013
 * \defgroup StatusLED StatusLED module
 *
 * This module is indicating multiple status of system at same time by the LED flashes.
 * Error type is specified according flash count.
 *
 * */

#include "FreeRTOS.h"
#include "CommonDefs.h"

//! count of the error kinds
#define ERROR_COUNT 10

//! limit number of recorded error per kind
#define ERROR_LIMIT 200

//! \brief Error code definitions - expresses number of flashes
/// \ingroup StatusLED
enum Err
{
	ERROR_OK = 1,
	ERROR_BATT = 2,
	ERROR_ACC = 3,
	ERROR_MOTOR = 4,
	ERROR_COMM_SLIP = 5,
	ERROR_COMM_CAN = 6,
	ERROR_COMMAND = 7
};

/*!
 * \brief StatusLEDInit module and task initialization
 *
 * Example usage:
 *  \code{c}
 	// StatusLed module and task initialization - this task have low priority
	if (StatusLEDInit(tskIDLE_PRIORITY +1) != pdPASS)
	{
		//there is error inside module initialization!
	}
	\endcode
 *
 * \param priority FreeRTOS priority at which the task should run
 * \ingroup StatusLED
 * \return pdPass value is returned if module task was created correctly
 * \warning This function require FreeRTOS environment
 */
signed long StatusLEDInit( unsigned long priority);


/*!
 * \brief Function for indication that new error appears
 *
 * Example usage:
 	 \code{c}
 	 // communication watchdog expired - we should inform user
 	 SetError(ERROR_COMM_SLIP);
 	 \endcode
 *
 * \ingroup StatusLED
 * \param err enum Err type error identifier
 */
void SetError( enum Err err);


/*!
 * \brief Function for indication that error disappears
 *
 * Example usage:
 	 \code{c}
 	 // communication refreshed - we should clear error
 	 ClearError(ERROR_COMM_SLIP);
 	 \endcode
 *
 * \ingroup StatusLED
 * \param err enum Err type error identifier
 */
void ClearError( enum Err err);

#endif//__STATUSLED_H__
