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
#include "RLS.h"


//! System clock initialization
void vSystemInit( void)
{
	//! CLOCK setup to 20 MHz - base frequency 200MHz
	SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_6MHZ);

	USER_SWITCH_INIT();

}

//! temporary motor mode
char motorMode = MOTOR_RUNNING;

//
void SlipSerialProcessPacket(char packet_buffer[], int length)
{
	//packet arrived
	switch (packet_buffer[0])
	{
	case ID_SET_SPEED:
		if (length<8)
		{
			//corrupted data
			// command error indication
			SetError(ERROR_COMMAND);
		}
		else
		{
			float speeds[2];
			// command error clearence
			ClearError(ERROR_COMM_SLIP);
			memcpy(&speeds,&packet_buffer[1],8);
			MotorControlSetSpeed(speeds[0],speeds[1]);
		}
		break;
	case ID_MOTOR_MODE:
		if (length<2)
		{
			//corrupted data
			// command error indication
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
				// command error indication
				SetError(ERROR_COMMAND);
			}
		}
		break;
	case ID_REG_PARAMS:
		if (length<6)
		{
			//corrupted data
			// command error indication
			SetError(ERROR_COMMAND);
		}
		else
		{
			// regulator structure update
			memcpy(&myDrive.mot1.reg.Kr,&packet_buffer[1],sizeof(float[6]));
			memcpy(&myDrive.mot2.reg.Kr,&packet_buffer[1],sizeof(float[6]));
		}
		break;
	case ID_GENERIC_COMMAND:
		switch (packet_buffer[1])
		{
		case COMMAND_GET_REG:
			//regulator parameters update
			SlipSend(ID_REG_PARAMS, (char *) &myDrive.mot1.reg.Kr, sizeof(float[3]));
			break;
		case COMMAND_RESET_ODOMETRY:
			// command activity
			MotorControlOdometryReset();
			break;
		}
		break;
	default:
		// command error indication
		SetError(ERROR_COMMAND);
		break;
	}
}

void SlipSerialReceiveTimeout( void)
{
	// timeout - stop motors to shutdown - prevent battery discharging
	MotorControlSetState(MOTOR_SHUTDOWN);
	// timeout error indication
	SetError(ERROR_COMM_SLIP);
}


void UserTask_task( void * param)
{
#if configUSE_TRACE_FACILITY==1
	// task list actualization prescaler
	short taskListPre = 100;
#endif
	// forewer
	while(1)
	{
		// now, we wait for new regulator sample period and new data
		if( MotorControlWaitData(50) == pdTRUE )
		{
			short regvalues[8];
			unsigned short values[3];
			char timestamp = 1;

			//acc values
			SlipSend(ID_ACC_STRUCT,(char *) &accData, 2*sizeof(short int));
			if ( AccelerometerRequestData() == pdPASS)
			{
				//OK
			}

			//regulator values copy
			regvalues[0] = myDrive.mot1.reg.measured;
			regvalues[1] = myDrive.mot2.reg.measured;
			regvalues[2] = myDrive.mot1.reg.action;
			regvalues[3] = myDrive.mot2.reg.action;
			regvalues[4] = myDrive.mot1.reg.error;
			regvalues[5] = myDrive.mot2.reg.error;
			regvalues[6] = myDrive.mot1.reg.desired;
			regvalues[7] = myDrive.mot2.reg.desired;

			// regulators data
			SlipSend(ID_REG,(char *) &regvalues, sizeof(short[8]));

			//data from ADC
			values[0] = myDrive.mot1.current_act;
			values[1] = myDrive.mot2.current_act;
			values[2] = myDrive.batt_voltage;

			//data from regulator
			SlipSend(ID_ADC,(char *) &values, sizeof(unsigned short[3]));

			//motor control state
			SlipSend(ID_MOTOR_MODE, (char *) &myDrive.state, sizeof(enum MotorState) );

			//odometry
			SlipSend(ID_POSITION, (char *) &myDrive.position, sizeof(float[3]));

#if MOTORCONTROL_IDENT_ENABLE == 1
			// identified parameters
			SlipSend(ID_IDENT_PARAMS, (char *) &ident.th->mat[0], sizeof(float[4]));
#endif
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

// user task initialization
signed portBASE_TYPE UserTaskInit( unsigned portBASE_TYPE priority )
{
	// we only create control task
	return xTaskCreate(UserTask_task, (signed portCHAR *) "USER", 512, NULL, priority , NULL);
}

int main(void)
{
	//! HW initialization
	vSystemInit();

	//! HeartBeat taks
	if (StatusLEDInit(tskIDLE_PRIORITY+1) != pdPASS)
	{
		//error
	}

	//! SlipSerial serial communication task
	if (SlipSerialInit(tskIDLE_PRIORITY +2, 57600, 600) != pdPASS)
	{
		//error
	}

	//! MotorControl module initialization
	if (MotorControlInit(tskIDLE_PRIORITY+4) != pdPASS)
	{
		// error occurred during MotorControl module initialization
	}

	//! Accelerometer task
	if (AccelerometerInit(tskIDLE_PRIORITY +3) != pdPASS)
	{
		//error
	}

	// task creation condition
	if (USER_SWITCH_READ2())
	{
		//! User control task initialization
		if ( UserTaskInit(tskIDLE_PRIORITY +2) != pdPASS)
		{
			//error
		}
	}

	// CANtest taks condition
	if (USER_SWITCH_READ3())
	{
		//! CANTest task
		if (CANtestInit(tskIDLE_PRIORITY +1) != pdPASS)
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

