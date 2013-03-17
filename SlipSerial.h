#ifndef  __SLIPSERIAL_H__
#define  __SLIPSERIAL_H__

#include "FreeRTOS.h"
#include "task.h"

enum SLIP_ID
{
	ID_ACC_STRUCT = 0x10,
	ID_REG = 0x11,
	ID_TASKLIST = 0x12,
	ID_ADC = 0x13,
	ID_MOTOR_ACT = 0x14,
	ID_MOTOR_MODE = 0x15,
	ID_TIME_STAMP = 0x16,
	ID_REG_PARAMS = 0x17
};

/*!
 * \brief SlipSerialInit
 *
 * Funkce Inicializuj�c� modul StatusLed
 *
 * \return [information about return value]
 * \sa [see also section]
 * \note [any note about the function you might have]
 * \warning [any warning if necessary]
 */
signed portBASE_TYPE SlipSerialInit( unsigned portBASE_TYPE priority, unsigned long int baudrate);

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
void SlipEnd( void);

/*!
 * \brief SlipSend
 *
 * Funkce Inicializuj�c� modul StatusLed
 *
 * \return [information about return value]
 * \sa [see also section]
 * \note [any note about the function you might have]
 * \warning [any warning if necessary]
 */
int SlipSend(char id, char * data, int len);

/*!
 * \brief UART1_IRQHandler
 *
 * Funkce Inicializuj�c� modul StatusLed
 *
 * \return [information about return value]
 * \sa [see also section]
 * \note [any note about the function you might have]
 * \warning [any warning if necessary]
 */
void UART1_IRQHandler( void);

/*!
 * \brief SlipSerial_task
 *
 * Funkce Inicializuj�c� modul StatusLed
 *
 * \return [information about return value]
 * \sa [see also section]
 * \note [any note about the function you might have]
 * \warning [any warning if necessary]
 */
void SlipSerial_task( void * param);

#endif//__SLIPSERIAL_H__
