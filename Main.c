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

/* kritická HOOK funkce indikující pøeteèení zásobníku nìkteré z úloh*/
void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
	( void ) pxTask;
	( void ) pcTaskName;
	GPIOPinWrite(LED_RED_PORT, LED_RED, ~LED_RED);
	for( ;; );
}

/* pro úèely ladìní Hard Fault - pøevzato z www.freertos. */
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

/* pro úèely ladìní Hard Fault - pøevzato z www.freertos.org */
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

int main(void)
{
	//! inicializace základních periferií systému
	SystemInit();

	//! HeartBeat úloha indikaèní diody
	StatusLEDInit();
	if (xTaskCreate(StatusLED_task, (signed portCHAR *) "LED", 256, NULL, tskIDLE_PRIORITY +1, NULL) != pdPASS)
	{
		//error
	}

	//! SlipSerial komunikace po seriové lince
	SlipSerialInit(57600);
	if (xTaskCreate(SlipSerial_task, (signed portCHAR *) "SLIP", 256, NULL, tskIDLE_PRIORITY +2, NULL) != pdPASS)
	{
		//error
	}

	//! úloha øízení motorù
	MotorControlInit();
	if (xTaskCreate(MotorControl_task, (signed portCHAR *) "MOTOR", 256, NULL, tskIDLE_PRIORITY +4, NULL) != pdPASS)
	{
		//error
	}

	//! úloha akcelerometru
	AccelerometrInit();
	if (xTaskCreate(Accelerometer_task, (signed portCHAR *) "ACC", 256, NULL, tskIDLE_PRIORITY +3, NULL) != pdPASS)
	{
		//error
	}

	//! CANTest úloha
	if (xTaskCreate(CANtest_task, (signed portCHAR *) "CAN", 256, NULL, tskIDLE_PRIORITY +1, NULL) != pdPASS)
	{
		//error
	}


#if configUSE_TRACE_FACILITY==1
	//! aktivace trasování objektù jádra OS
	if (uiTraceStart() != 1)
		while(1){};
#endif

	//! spuštìní plánovaèe úloh freeRTOS
	vTaskStartScheduler();

	//! sem se nesmíme dostat!
	GPIOPinWrite(LED_RED_PORT, LED_RED, ~LED_RED);
	while(1){};
	return 0;
}
