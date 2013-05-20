
#include "CommonDefs.h"
#include "StatusLED.h"


/*! \file CANtest.h
 * \author Lukas Otava
 * \date 2013
 * \defgroup CANtest CANtest module
 *
 * This module test CAN communication interface.
 *
 * */

/*! \brief This CANtest module initialization function
 *
 * Example usage:
 *  \code{c}
 *  // CANTest task
	if (xTaskCreate(CANtest_task, (signed portCHAR *) "CAN", 256, NULL, tskIDLE_PRIORITY +1, NULL) != pdPASS)
	{
		//error
	}
	\endcode
 * \ingroup CANtest
 *  */
signed long CANtestInit( unsigned long priority);


/* CANtest FreeRTOS task */
void CANtest_task( void * param);
