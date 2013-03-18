

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

//! number of recorded errors
#define ERROR_COUNT 10

//! limit value of recorded error
#define ERROR_LIMIT 200

//! array of errors
short errors[ERROR_COUNT];

signed portBASE_TYPE StatusLEDInit( unsigned portBASE_TYPE priority)
{
	short i;

	//! init GPIO
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(LED_RED_PORT,LED_RED);//LED

	//! default values
	for (i = 0; i < ERROR_COUNT; i++)
	{
		errors[i] = 0;
	}

	//! add default singel flash error
	errors[ERROR_OK] = 1;

	//! task creation
	return xTaskCreate(StatusLED_task, (signed portCHAR *) "LED", 256, NULL, priority, NULL);

}

void StatusLED_task( void * pvParameters )
{
	//! general variables
	//short prescaler = 0;
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
					//! code for one flash
					GPIOPinWrite(LED_RED_PORT, LED_RED, ~LED_RED);
					vTaskDelay(80);
					GPIOPinWrite(LED_RED_PORT, LED_RED, LED_RED);
					vTaskDelay(320);
				}
				// time delay between every error flashes
				vTaskDelay(1200);
			}
		}

	}
}

void SetError( enum Err err)
{
	//! for valid error number
	if (err< ERROR_COUNT)
		//! increment error occures
		if (errors[err]<ERROR_LIMIT)
			errors[err]++;
}

void ClearError( enum Err err)
{
	//! NEED TO BE ADDED
	//! for valid error number
	if (err< ERROR_COUNT)
		//! clear all error occures
		if (errors[err]>0)
			errors[err] = 0;
}
