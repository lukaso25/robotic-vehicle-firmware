#ifndef  __MOTORCONTROL_H__
#define  __MOTORCONTROL_H__

/*! \file MotorControl.h
 * \author Lukas Otava
 * \date 2013
 * \defgroup MotorControl MotorControl module
 *
 * Motor control module use the PWM module and the QEI module of microcontroller for speed control of two motors.
 *
 * */

#include "CommonDefs.h"
#include "Regulator.h"
#include "FreeRTOS.h"

#define MOTOR_PWM_PERIOD		(4000) //4000 pro 20 MHz 5 kHz //4000 pro 40MHz a 10kHz
#define SPEED_REG_PERIOD		((20000000/50)-1) //pøednastavení QEI èasovaèe pro periodu 20ms (50Hz)

#define SUM_LIMIT (1000)

/*! \brief Available states of MotorControl module
 * \ingroup MotorControl */
enum MotorState
{
	//! in this state H-bridge is in Hi-Z
	MOTOR_SHUTDOWN = 0,
	MOTOR_STOP = 1,
	//! normal operation
	MOTOR_RUNNING  = 2,
	MOTOR_MANUAL = 3,
	MOTOR_FAILURE = 4
};

enum RegID
{
	MOTOR_1 = 0, MOTOR_2 = 1
};

struct SensorActor
{
	char id;
	short value;
};

/*! \brief This structure holds run-time information and parameters of the motor regulator
 * \ingroup MotorControl */
struct MotorControl
{

	struct RegulatorParams reg;

	//stats
	//int speed_avg;
	//short speed_max;
	//long track;

	unsigned short current_act;
	unsigned long current_total;

	short qei_errors;

};

/*! \brief This structure holds run-time information and parameters of the motor regulators and position
 * \ingroup MotorControl */
struct DriveBlock
{
	struct MotorControl mot1;
	struct MotorControl mot2;

	struct Position position;

	enum MotorState state;
};


/*! \brief This structure holds run-time information and parameters of the drive
 * \ingroup MotorControl */
struct DriveBlock myDrive;
//struct RegulatorParams myReg;

/*! \brief This function initialize MotorControl module and task
 *
 *  Example usage:
 *  \code{c}
	//MotorControl module initialization
	if (MotorControlInit(tskIDLE_PRIORITY+4) != pdPASS)
	{
		// error occurred during MotorControl module initialization!
	}
	\endcode
 * \param priority FreeRTOS priority at which the task should run
 * \ingroup MotorControl
 * \return pdPass value is returned if module task was created correctly
 * \warning This function require FreeRTOS environment */
signed portBASE_TYPE MotorControlInit( unsigned portBASE_TYPE priority);

/*! \brief
 *
 * Example usage:
 *  \code{c}

	\endcode
 * \param
 * \ingroup MotorControl
 * \return
 * \warning */
void MotorControlSetState( enum MotorState st);

/*! \brief
 *
 * Example usage:
 *  \code{c}

	\endcode
 * \param
 * \ingroup MotorControl
 * \return
 * \warning */
enum MotorState MotorControlGetState( void);

signed portBASE_TYPE MotorControlWaitData(portTickType timeout);

/*! \brief This function can set a speed for both motors
 *
 * The input speed is signed short integer. Dimension of this speed is incremental sensor ticks peer sampling period.
 * eg. \f$ [v_1 =  \frac{ticks}{period}\f$
 *
 * Example usage:
 *  \code{c}

	\endcode
 * \param v1 desired speed for motor 1
 * \param v2 desired speed for motor 2
 * \ingroup MotorControl
 * */
void MotorControlSetSpeed(signed short v1, signed short v2);


// FreeRTOS task
void MotorControl_task( void * param);

#endif//__MOTORCONTROL_H__
