
#include "StatusLED.h"

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"


#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "timers.h"

#include "trcUser.h"

#include "CommonDefs.h"
#include "SlipSerial.h"

#define ERROR_COUNT 10
#define ERROR_LIMIT 200

short errors[ERROR_COUNT];

// inicializace GPIO LED
void StatusLEDInit( void)
{
	short i;
	//! init GPIO
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(LED_RED_PORT,LED_RED);//LED

	for (i = 0; i < ERROR_COUNT; i++)
	{
		errors[i] = 0;
	}
	errors[ERROR_OK] = 1;
}

//! �loha indika�n� led a v�pisu �loh
void StatusLED_task( void * pvParameters )
{
	short prescaler = 0;
	short i;
	short j;

	//! never-ending loop
	while(1)
	{
		//projdi jednotliv� chybov� �rovn�
		for (j = 1; j < ERROR_COUNT; j++)
		{
			// vyblikat k�d?
			if (errors[j]>0)
			{
				// po�et bliknut�
				for (i = 0; i < j; i++)
				{
					GPIOPinWrite(LED_RED_PORT, LED_RED, ~LED_RED);
					vTaskDelay(80);
					GPIOPinWrite(LED_RED_PORT, LED_RED, LED_RED);
					vTaskDelay(320);
				}
				//�ek�n�
				vTaskDelay(1200);
			}
			else
			{
				continue;
			}
		}

		//p�edd�li�ka odes�l�n� v�pisu �loh
		if (prescaler-- == 0)
		{
			prescaler = 2;
#if configUSE_TRACE_FACILITY==1
			char tasklist[256];
			vTaskList((signed char*)tasklist);
			SlipSend(ID_TASKLIST,tasklist, strlen(tasklist));
#endif
		}

	}
}

//! zaznamen�n� chyby
void StatusLEDSetError( enum Err err)
{
	if (err< ERROR_COUNT)
		if (errors[err]<ERROR_LIMIT)
			errors[err]++;
}

//! zru�en� zobrazen� chyby
void StatusLEDClearError( enum Err err)
{
	if (err< ERROR_COUNT)
		if (errors[err]>0)
			errors[err] = 0;
}
