#ifndef  __MOTORCONTROL_H__
#define  __MOTORCONTROL_H__

#include "CommonDefs.h"
#include "Regulator.h"
#include "FreeRTOS.h"

#define MOTOR_PWM_PERIOD		(4000) //4000 pro 20 MHz 5 kHz //4000 pro 40MHz a 10kHz
#define SPEED_REG_PERIOD		((20000000/50)-1) //pøednastavení QEI èasovaèe pro periodu 20ms (50Hz)

#define SUM_LIMIT (1000)


enum MotorState
{
	MOTOR_SHUTDOWN = 0, MOTOR_STOP = 1, MOTOR_RUNNING  = 2, MOTOR_MANUAL = 3, MOTOR_FAILURE = 4
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

struct DriveBlock
{
	struct MotorControl mot1;
	struct MotorControl mot2;

	struct Position position;

	enum MotorState state;
};

struct DriveBlock myDrive;
//struct RegulatorParams myReg;


signed portBASE_TYPE MotorControlInit( unsigned portBASE_TYPE priority);


void MotorControlSetState( enum MotorState st);
enum MotorState MotorControlGetState( void);

signed portBASE_TYPE MotorControlWaitData(portTickType timeout);

//! FreeRTOS task
void MotorControl_task( void * param);

#endif//__MOTORCONTROL_H__
