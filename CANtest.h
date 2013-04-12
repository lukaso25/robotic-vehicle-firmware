/*! \file CANtest.h
 * \author Lukas Otava
 * \date 2013
 * \defgroup CANtest CANtest module
 *
 * This module test CAN communication interface.
 *
 * */

/*! \brief This function changes between Motor control module modes
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
void CANtest_task( void * param);
