#ifndef  __SLIPSERIAL_H__
#define  __SLIPSERIAL_H__

#include "FreeRTOS.h"

// maximální delka pøíjmaného paketu
#define MAX_PACKET_LENGH	(128)

enum SLIP_ID
{
	ID_ACC_STRUCT = 0x10,
	ID_REG = 0x11,
	ID_TASKLIST = 0x12,
	ID_ADC = 0x13,
	ID_MOTOR_ACT = 0x14,
	ID_MOTOR_MODE = 0x15,
	ID_TIME_STAMP = 0x16,
	ID_REG_PARAMS = 0x17,
	ID_IDENT_PARAMS = 0x18,
	ID_GENERIC_COMMAND = 0x1A,
	ID_STRING = 0x1B,
	ID_POSITION = 0x1C,
	ID_SET_SPEED = 0x1D
};

enum GENERIC_COMMAND_ID
{
	COMMAND_NONE = 0x0,
	COMMAND_ADAPTIVE_REG,
	COMMAND_FIXED_REG
};

/*!
 * \brief Funkce volaná pøi korektním pøijmu packetu
 *
 * \param packet_buffer[] pole pøijatých bajtù
 * \param length velikost paketu
 * \return void
 * \note Tato funkce musí být implementována v jiném modulu
 * \warning využívá FreeRTOS
 */
extern void SlipSerialProcessPacket(char packet_buffer[], int length);

/*!
 * \brief Funkce volaná pøi vypršení nastaveného timeoutu
 *
 * \note Tato funkce musí být implementována v jiném modulu
 * \warning využívá FreeRTOS
 */
extern void SlipSerialReceiveTimeout( void);

/*!
 * \brief SlipSerialInit
 *
 * \return [information about return value]
 * \note [any note about the function you might have]
 * \warning využívá FreeRTOS
 */
signed portBASE_TYPE SlipSerialInit( unsigned portBASE_TYPE priority, unsigned long int baudrate, portTickType timeout);

/*!
 * \brief
 *
 * \return [information about return value]
 * \sa [see also section]
 * \note [any note about the function you might have]
 * \warning [any warning if necessary]
 */
void SlipEnd( void);

/*!
 * \brief
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
 * \return [information about return value]
 * \sa [see also section]
 * \note [any note about the function you might have]
 * \warning [any warning if necessary]
 */
void UART1_IRQHandler( void);

/*!
 * \brief SlipSerial_task
 *
 *
 * \return [information about return value]
 * \sa [see also section]
 * \note [any note about the function you might have]
 * \warning [any warning if necessary]
 */
void SlipSerial_task( void * param);

#endif//__SLIPSERIAL_H__
