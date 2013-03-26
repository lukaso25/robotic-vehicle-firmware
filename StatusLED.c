

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

#include "CommonDefs.h"

//! array of errors
short errors[ERROR_COUNT];


/*
 * \brief StatusLED module FreeRTOS task
 */
void StatusLED_task( void * pvParameters );

signed portBASE_TYPE StatusLEDInit( unsigned portBASE_TYPE priority)
{
	// temporary variable
	short i;

	// GPIO module initialization, pin connection
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(LED_RED_PORT,LED_RED);

	// default values for array of errors
	for (i = 0; i < ERROR_COUNT; i++)
	{
		errors[i] = 0;
	}

	// add default single flash - running indicator - heard beat
	errors[ERROR_OK] = 1;

	// finally we create task
	return xTaskCreate(StatusLED_task, (signed portCHAR *) "LED", 256, NULL, priority, NULL);
}

void StatusLED_task( void * pvParameters )
{
	// general variables
	short i;
	short j;

	// never-ending task loop
	while(1)
	{
		// for entire error kinds
		for (j = 1; j < ERROR_COUNT; j++)
		{
			// is error present?
			if (errors[j]>0)
			{
				// how many flashes?
				for (i = 0; i < j; i++)
				{
					// code for one flash
					GPIOPinWrite(LED_RED_PORT, LED_RED, ~LED_RED);
					vTaskDelay(80);
					GPIOPinWrite(LED_RED_PORT, LED_RED, LED_RED);
					vTaskDelay(320);
				}

				// time delay between every flash
				vTaskDelay(1200);
			}
		}
	}
}

void SetError( enum Err err)
{
	// for valid error number
	if (err< ERROR_COUNT)
		// increment error occures
		if (errors[err]<ERROR_LIMIT)
			errors[err]++;
}

void ClearError( enum Err err)
{
	//! NEED TO BE ADDED
	// for valid error number
	if (err< ERROR_COUNT)
		// clear all error occures
		if (errors[err]>0)
			errors[err] = 0;
}
