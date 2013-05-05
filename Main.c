#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"

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
	case ID_GENERIC_COMMAND:
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

void ControlTask_task( void * param)
{
#if configUSE_TRACE_FACILITY==1
	short taskListPre = 100;
#endif
	rlseType ident1;
	//rlseType ident2;
	regParamType regpar;

	rlse_init(&ident1);
	//default parameters
	ident1.th->mat[0] = -1.43380;
	ident1.th->mat[1] = 0.44817;
	ident1.th->mat[2] = -0.64161;
	ident1.th->mat[3] = 0.64161;

	//rlse_init(&ident2);

	while(1)
	{
		if( MotorControlWaitData(50) == pdTRUE )
		{
			short regvalues[6];
			unsigned short values[3];
			char timestamp = 1;

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
			SlipSend(ID_IDENT_PARAMS, (char *) &ident1.th->mat[0], 4*sizeof(float));

			//timestamp
			SlipSend(ID_TIME_STAMP,(char *) &timestamp, sizeof(char));

			rlse_update( &ident1, (float) regvalues[0],(float) regvalues[2], ((regvalues[4]>400)||(regvalues[4]<-400)));
			//rmnc_update( &ident2, (float) regvalues[1],(float) regvalues[3], (regvalues[1]!= 0));

			compute_params(ident1.th,&regpar);

			if ((regpar.Kr > 0.1) && (regpar.Kr < 1))
			{
				RegulatorSetPID(&myDrive.mot1.reg,regpar.Kr,regpar.Ti,regpar.Td);
				RegulatorSetPID(&myDrive.mot2.reg,regpar.Kr,regpar.Ti,regpar.Td);

				//regulator params update
				SlipSend(ID_REG_PARAMS, (char *) &myDrive.mot1.reg.Kr, 3*sizeof(float));
			}
			else
			{
				RegulatorSetPID(&myDrive.mot1.reg, 0.5, 0.19, 1.3);
				RegulatorSetPID(&myDrive.mot2.reg, 0.5, 0.19, 1.3);
			}
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

