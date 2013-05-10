#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"

#include <math.h>

//#include "semihosting.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "timers.h"

#include "trcUser.h"

#include "CommonDefs.h"
#include "StatusLED.h"
#include "Accelerometer.h"
#include "Motorcontrol.h"
#include "SlipSerial.h"
#include "CANtest.h"
#include "RLSE.h"


void vSystemInit( void)
{
	//! CLOCK setup to 20 MHz - base frequency 200MHz
	SysCtlClockSet(SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_6MHZ);

	USER_SWITCH_INIT();

}

//! temporary motor mode
char motorMode = MOTOR_RUNNING;

char namiste = 0;
struct Position desPos;

void SlipSerialProcessPacket(char packet_buffer[], int length)
{
	//packet arrived
	switch (packet_buffer[0])
	{
	case ID_MOTOR_ACT:
		if (length<5)
		{
			//corrupted data
			SetError(ERROR_COMMAND);
		}
		else
		{
			ClearError(ERROR_COMM_SLIP);
			MotorControlSetWheelSpeed(packet_buffer[1]|(packet_buffer[2]<<8), packet_buffer[3]|(packet_buffer[4]<<8));
		}
		break;
	case ID_SET_SPEED:
		if (length<8)
		{
			//corrupted data
			SetError(ERROR_COMMAND);
		}
		else
		{
			float speeds[2];
			ClearError(ERROR_COMM_SLIP);
			memcpy(&speeds,&packet_buffer[1],8);
			MotorControlSetSpeed(speeds[0],speeds[1]);
			//MotorControlSetState(motorMode);
		}
		break;
	case ID_MOTOR_MODE:
		if (length<2)
		{
			//corrupted data
			SetError(ERROR_COMMAND);
		}
		else
		{
			if (packet_buffer[1]<6)
			{
				motorMode = packet_buffer[1];
				MotorControlSetState(motorMode);
			}
			else
			{
				SetError(ERROR_COMMAND);
			}
		}
		break;
	case ID_REG_PARAMS:
		if (length<6)
		{
			//corrupted data
			SetError(ERROR_COMMAND);
		}
		else
		{
			memcpy(&myDrive.mot1.reg.Kr,&packet_buffer[1],24);
			memcpy(&myDrive.mot2.reg.Kr,&packet_buffer[1],24);
		}
		break;
	case ID_POSITION:
		if (length<6)
		{
			//corrupted data
			SetError(ERROR_COMMAND);
		}
		else
		{
			memcpy(&desPos,&packet_buffer[1],12);
		}
		break;
	case ID_GENERIC_COMMAND:
		switch (packet_buffer[1])
		{
		case COMMAND_GO:
			//namiste = 0;
			break;
		case COMMAND_GET_REG:
			//regulator params update
			SlipSend(ID_REG_PARAMS, (char *) &myDrive.mot1.reg.Kr, 3*sizeof(float));
			break;
		case COMMAND_ADAPTIVE_REG:
			MotorControlSetSelfTuning(SELFTUNUNG_START);
			break;
		case COMMAND_RESET_ODOMETRY:
			MotorControlOdometryReset();
			break;
		}
		break;
	default:
		SetError(ERROR_COMMAND);
		break;
	}
}

void SlipSerialReceiveTimeout( void)
{
	//  timeout
	MotorControlSetState(MOTOR_SHUTDOWN);
	SetError(ERROR_COMM_SLIP);
}

/*float AngleError(struct Position* actual, struct Position* desired)
{
	float error = 0.0;

	float zu = atan2f((desired->y-actual->y),(desired->x-actual->x));
	float th = atan2f(sinf(actual->theta),cosf(actual->theta));

	error = zu - th;
	SlipSend(ID_TEST_PARAM, (char *) &error, sizeof(float));

	return error;
}
float Distance(struct Position* actual, struct Position* desired)
{
	return sqrtf( (desired->y-actual->y)*(desired->y-actual->y) + (desired->x-actual->x)*(desired->x-actual->x));
}*/

void ControlTask_task( void * param)
{
#if configUSE_TRACE_FACILITY==1
	short taskListPre = 100;
#endif

	//short posreg = 5;

	//new point
	desPos.x = 100;
	desPos.y = 100;

	while(1)
	{
		if( MotorControlWaitData(50) == pdTRUE )
		{
			short regvalues[6];
			unsigned short values[3];
			char timestamp = 1;

			/*posreg--;
			if (posreg == 0)
			{
				posreg = 5;
				if (!namiste)
				{
					MotorControlSetSpeed(100.0,0.5*AngleError(&desPos,&myDrive.position));
					if (Distance(&desPos,&myDrive.position) < 10.0)
						namiste = 1;
				}
				else
				{
					MotorControlSetSpeed(0.0,0.0);
				}

			}*/

			//acc values
			SlipSend(ID_ACC_STRUCT,(char *) &accData, 2*sizeof(short int));
			if ( AccelerometerRequestData() == pdPASS)
			{
				//OK
			}

			//regulator values
			regvalues[0] = myDrive.mot1.reg.measured;
			regvalues[2] = myDrive.mot1.reg.action;
			regvalues[4] = myDrive.mot1.reg.error;
			regvalues[1] = myDrive.mot2.reg.measured;
			regvalues[3] = myDrive.mot2.reg.action;
			regvalues[5] = myDrive.mot2.reg.error;

			SlipSend(ID_REG,(char *) &regvalues, sizeof(short[6]));

			//data from ADC
			values[0] = myDrive.mot1.current_act;
			values[1] = myDrive.mot2.current_act;
			values[2] = myDrive.batt_voltage;

			//data from regulator
			SlipSend(ID_ADC,(char *) &values, sizeof(unsigned short[3]));

			//motor control state
			SlipSend(ID_MOTOR_MODE, (char *) &myDrive.state, sizeof(enum MotorState) );

			//odometry
			SlipSend(ID_POSITION, (char *) &myDrive.position, 3*sizeof(float));

			// identified parameters
			SlipSend(ID_IDENT_PARAMS, (char *) &ident.th->mat[0], 4*sizeof(float));

			SlipSend(ID_TEST_PARAM, (char *) &myDrive.mot1.reg.sum, sizeof(float));

			//timestamp
			SlipSend(ID_TIME_STAMP,(char *) &timestamp, sizeof(char));

		}
		else
		{
			//timeout
		}

#if configUSE_TRACE_FACILITY==1
		// TASK LIST  - this block communication
		taskListPre--;
		if (taskListPre == 0)
		{
			taskListPre = 100;

			char tasklist[256];
			vTaskList((signed char*)tasklist);
			SlipSend(ID_TASKLIST,tasklist, strlen(tasklist));
		}
#endif
	}
}

signed portBASE_TYPE ControlTaskInit( unsigned portBASE_TYPE priority )
{
	// we only create control task
	return xTaskCreate(ControlTask_task, (signed portCHAR *) "CTRL", 512, NULL, priority , NULL);
}



int main(void)
{
	//! inicializace základních periferií systému
	vSystemInit();

	//! HeartBeat úloha indikaèní diody
	if (StatusLEDInit(tskIDLE_PRIORITY+1) != pdPASS)
	{
		//error
	}

	//! SlipSerial komunikace po seriové lince
	if (SlipSerialInit(tskIDLE_PRIORITY +2, 57600, 600) != pdPASS)
	{
		//error
	}

	//! MotorControl module initialization
	if (MotorControlInit(tskIDLE_PRIORITY+4) != pdPASS)
	{
		// error occurred during MotorControl module initialization
	}

	//! úloha akcelerometru
	if (AccelerometerInit(tskIDLE_PRIORITY +3) != pdPASS)
	{
		//error
	}

	if (USER_SWITCH_READ2())
	{
		//! úloha øízení  - tady budou kraviny kolem, logování atd
		if ( ControlTaskInit(tskIDLE_PRIORITY +2) != pdPASS)
		{
			//error
		}
	}

	if (USER_SWITCH_READ3())
	{
		//! CANTest task
		if (xTaskCreate(CANtest_task, (signed portCHAR *) "CAN", 256, NULL, tskIDLE_PRIORITY +1, NULL) != pdPASS)
		{
			//error
		}
	}

#if configUSE_TRACE_FACILITY==1
	//! FreeRTOs kernel object trace record start
	if (uiTraceStart() != 1)
		while(1){};
#endif

	//! FreeRTOS scheduler start
	vTaskStartScheduler();

	//! never entering code
	while(1){};
	return 0;
}

