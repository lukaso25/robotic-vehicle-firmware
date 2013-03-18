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


void SystemInit( void)
{
	//! CLOCK setup to 20 MHz
	SysCtlClockSet(SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_6MHZ);

}

/*static xQueueHandle xErrMess;
void Status_task( void * params)
{

}*/

/* kritick� HOOK funkce indikuj�c� p�ete�en� z�sobn�ku n�kter� z �loh*/
void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
	( void ) pxTask;
	( void ) pcTaskName;
	GPIOPinWrite(LED_RED_PORT, LED_RED, ~LED_RED);
	for( ;; );
}

/* pro ��ely lad�n� Hard Fault - p�evzato z www.freertos. */
void prvGetRegistersFromStack( uint32_t *pulFaultStackAddress )
{

	/* These are volatile to try and prevent the compiler/linker optimising them
	away as the variables never actually get used.  If the debugger won't show the
	values of the variables, make them global my moving their declaration outside
	of this function. */
	volatile uint32_t r0;
	volatile uint32_t r1;
	volatile uint32_t r2;
	volatile uint32_t r3;
	volatile uint32_t r12;
	volatile uint32_t lr; /* Link register. */
	volatile uint32_t pc; /* Program counter. */
	volatile uint32_t psr;/* Program status register. */

	r0 = pulFaultStackAddress[ 0 ];
	r1 = pulFaultStackAddress[ 1 ];
	r2 = pulFaultStackAddress[ 2 ];
	r3 = pulFaultStackAddress[ 3 ];

	r12 = pulFaultStackAddress[ 4 ];
	lr = pulFaultStackAddress[ 5 ];
	pc = pulFaultStackAddress[ 6 ];
	psr = pulFaultStackAddress[ 7 ];

	/* When the following line is hit, the variables contain the register values. */
	for( ;; );

	r0;
	r1;
	r2;
	r3;
	r12;
	lr;
	pc;
	psr;
}

/* pro ��ely lad�n� Hard Fault - p�evzato z www.freertos.org */
/* The prototype shows it is a naked function - in effect this is just an
assembly function. */
__attribute__( ( naked ) ) void HardFault_Handler(void)
{
	__asm volatile
	(
			" tst lr, #4                                                \n"
			" ite eq                                                    \n"
			" mrseq r0, msp                                             \n"
			" mrsne r0, psp                                             \n"
			" ldr r1, [r0, #24]                                         \n"
			" ldr r2, handler2_address_const                            \n"
			" bx r2                                                     \n"
			" handler2_address_const: .word prvGetRegistersFromStack    \n"
	);
}

//! re�im motoru
char motorMode = MOTOR_RUNNING;

void SlipSerialProcessPacket(char packet_buffer[], int length)
{
	switch (packet_buffer[0])
	{
	case ID_MOTOR_ACT:
		//m�me p�ijato :-)
		if (length<5)
		{
			//po�kozen� data
		}
		else
		{
			//need to be tested
		}
		ClearError(ERROR_COMM_SLIP);
		myDrive.mot1.reg.desired = packet_buffer[1]|(packet_buffer[2]<<8);
		myDrive.mot2.reg.desired = packet_buffer[3]|(packet_buffer[4]<<8);
		MotorControlSetState(motorMode);
		break;
	case ID_MOTOR_MODE:
		if (length<2)
		{
			//po�kozen� data
		}
		motorMode = packet_buffer[1];
		MotorControlSetState(motorMode);
		break;
	case ID_REG_PARAMS:
		if (length<6)
		{
			//po�kozen� data
		}
		memcpy(&myDrive.mot1.reg.K,&packet_buffer[1],8);
		memcpy(&myDrive.mot2.reg.K,&packet_buffer[1],8);
		break;
	}
}

void SlipSerialReceiveTimeout( void)
{
	MotorControlSetState(MOTOR_SHUTDOWN);
	SetError(ERROR_COMM_SLIP);
}

void ControlTask_task( void * param)
{
	while(1)
	{

#if configUSE_TRACE_FACILITY==1
			char tasklist[256];
			vTaskList((signed char*)tasklist);
			SlipSend(ID_TASKLIST,tasklist, strlen(tasklist));
#endif
		vTaskDelay(2000);
	}
}

signed portBASE_TYPE ControlTaskInit( unsigned portBASE_TYPE priority)
{

	return xTaskCreate(ControlTask_task, (signed portCHAR *) "CTRL", 256, NULL, priority , NULL);
}



int main(void)
{
	//! inicializace z�kladn�ch periferi� syst�mu
	SystemInit();

	//! HeartBeat �loha indika�n� diody
	if (StatusLEDInit(tskIDLE_PRIORITY +1) != pdPASS)
	{
		//error
	}

	//! SlipSerial komunikace po seriov� lince
	if (SlipSerialInit(tskIDLE_PRIORITY +2, 57600, 200) != pdPASS)
	{
		//error
	}

	//! �loha ��zen� motor�
	if (MotorControlInit(tskIDLE_PRIORITY +4) != pdPASS)
	{
		//error
	}

	//! �loha akcelerometru
	if (AccelerometrInit(tskIDLE_PRIORITY +3) != pdPASS)
	{
		//error
	}

	//! �loha ��zen�  - tady budou kraviny kolem, logov�n� atd
	if ( ControlTaskInit(tskIDLE_PRIORITY +2) != pdPASS)
	{
		//error
	}

	//! CANTest �loha
	if (xTaskCreate(CANtest_task, (signed portCHAR *) "CAN", 256, NULL, tskIDLE_PRIORITY +1, NULL) != pdPASS)
	{
		//error
	}


#if configUSE_TRACE_FACILITY==1
	//! aktivace trasov�n� objekt� j�dra OS
	if (uiTraceStart() != 1)
		while(1){};
#endif

	//! spu�t�n� pl�nova�e �loh freeRTOS
	vTaskStartScheduler();

	//! sem se nesm�me dostat!
	GPIOPinWrite(LED_RED_PORT, LED_RED, ~LED_RED);
	while(1){};
	return 0;
}
