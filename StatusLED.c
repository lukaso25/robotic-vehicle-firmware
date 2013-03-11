
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

//! úloha indikaèní led a výpisu úloh
void StatusLED_task( void * pvParameters )
{
	short prescaler = 0;
	short i;
	short j;

	//! never-ending loop
	while(1)
	{
		//projdi jednotlivé chybové úrovnì
		for (j = 1; j < ERROR_COUNT; j++)
		{
			// vyblikat kód?
			if (errors[j]>0)
			{
				// poèet bliknutí
				for (i = 0; i < j; i++)
				{
					GPIOPinWrite(LED_RED_PORT, LED_RED, ~LED_RED);
					vTaskDelay(80);
					GPIOPinWrite(LED_RED_PORT, LED_RED, LED_RED);
					vTaskDelay(320);
				}
				//èekání
				vTaskDelay(1200);
			}
			else
			{
				continue;
			}
		}

		//pøeddìlièka odesílání výpisu úloh
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

//! zaznamenání chyby
void StatusLEDSetError( enum Err err)
{
	if (err< ERROR_COUNT)
		if (errors[err]<ERROR_LIMIT)
			errors[err]++;
}

//! zrušení zobrazení chyby
void StatusLEDClearError( enum Err err)
{
	if (err< ERROR_COUNT)
		if (errors[err]>0)
			errors[err] = 0;
}
