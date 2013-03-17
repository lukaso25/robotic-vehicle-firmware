#ifndef __STATUSLED_H__
#define __STATUSLED_H__

#include "FreeRTOS.h"

// HW definitions
#define LED_RED			(GPIO_PIN_2)
#define LED_RED_PORT	(GPIO_PORTF_BASE)


/*!
 * \brief StatusLEDInit
 *
 * Funkce Inicializuj�c� modul StatusLed
 *
 * \return [information about return value]
 * \sa [see also section]
 * \note [any note about the function you might have]
 * \warning [any warning if necessary]
 */
signed portBASE_TYPE StatusLEDInit( unsigned portBASE_TYPE priority);


enum Err
{
	ERROR_OK = 1,
	ERROR_BATT = 2,
	ERROR_ACC = 3,
	ERROR_MOTOR = 4,
	ERROR_COMM_SLIP = 5,
	ERROR_COMM_CAN = 6
};



/*!
 * \brief �loha indika�n� led a v�pisu �loh
 *
 * Funkce zav�d�j�c� modul StatusLed do
 *
 * \return [information about return value]
 * \sa [see also section]
 * \note [any note about the function you might have]
 * \warning [any warning if necessary]
 */
void StatusLED_task( void * pvParameters );


/*!
 * \brief zaznamen�n� chyby
 *
 *
 * \return [information about return value]
 * \sa [see also section]
 * \note [any note about the function you might have]
 * \warning [any warning if necessary]
 */
void SetError( enum Err err);


/*!
 * \brief zru�en� zobrazen� chyby
 *
 *
 *
 * \return [information about return value]
 * \sa [see also section]
 * \note [any note about the function you might have]
 * \warning [any warning if necessary]
 */
void ClearError( enum Err err);

#endif//__STATUSLED_H__
