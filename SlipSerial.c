#include "SlipSerial.h"

#include "StatusLED.h"
#include "MotorControl.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "trcUser.h"

#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"


// SLIP makra znak�
#define SLIP_END             0xC0    /* indicates end of packet */
#define SLIP_ESC             0xDB    /* indicates byte stuffing */
#define SLIP_ESC_END         0xDC    /* ESC ESC_END means END data byte */
#define SLIP_ESC_ESC         0xDD    /* ESC ESC_ESC means ESC data byte */

// maxim�ln� delka p��jman�ho paketu
#define MAX_PACKET_LENGH	(300)


// OS objects objekty OS
xSemaphoreHandle xTxMutex;
char packet_buffer[MAX_PACKET_LENGH];
static xQueueHandle xSerialReceive;

//! inicializa�n� funkce
signed portBASE_TYPE SlipSerialInit( unsigned portBASE_TYPE priority, unsigned long int baudrate, portTickType timeout)
{
	//! function return value
	signed portBASE_TYPE error;

	//! povolen� hodin UART and GPIO module
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);


	//! propojen� v�vod� s jednotkou
	GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_2 | GPIO_PIN_3);

	//! Inicializace form�tu a rychlosti
	UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 57600,
			(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

	// Povolen� p�eru�en� a jeho nastaven� priority
	UARTIntEnable(UART1_BASE, UART_INT_RX);

	// nastaven� priority p�eru�en�
	IntPrioritySet(INT_UART1,(6<<5));
	IntEnable(INT_UART1);

	// vytvo�en� p�ij�mac� fronty
	xSerialReceive = xQueueCreate( 32, ( unsigned portBASE_TYPE ) sizeof( char ) );

	//! Povolen� UART.
	UARTEnable(UART1_BASE);


	xTxMutex = xSemaphoreCreateMutex();
	//vSemaphoreCreateBinary(xTxMutex); //- from interrupt

	// vytvo��me
	error = xTaskCreate(SlipSerial_task, (signed portCHAR *) "SLIP", 256, NULL, tskIDLE_PRIORITY +2, NULL);

	// ORAVIT
	if (xTxMutex != NULL)
	{
		return error; // sucess
	}
	else
	{
		return pdFAIL;// fail
	}

}

//! odesl�n� ukon�ovac�ho znaku
void SlipEnd( void)
{
	UARTCharPut(UART1_BASE,SLIP_END);
}

//! odesl�n� bufferu jako slip paket
int SlipSend(char id, char * data, int len)
{
	int temp = len;
	if( xSemaphoreTake( xTxMutex, ( portTickType ) 100 ) != pdTRUE )
	{
		return -1; //error - mutex expiration
	}

	UARTCharPut(UART1_BASE,SLIP_END);
	switch (id)
	{
	case SLIP_END: //end occours
		UARTCharPut(UART1_BASE,SLIP_ESC);
		UARTCharPut(UART1_BASE,SLIP_ESC_END);
		break;
	case SLIP_ESC:
		UARTCharPut(UART1_BASE,SLIP_ESC);
		UARTCharPut(UART1_BASE,SLIP_ESC_ESC);
		break;
	default:
		UARTCharPut(UART1_BASE,id);
	}

	while ( (len--) > 0)
	{
		switch ((unsigned char)*data)
		{
		// if it's the same code as an END character, we send a special two character code so as not to make the receiver think we sent an END
		case SLIP_END:
			UARTCharPut(UART1_BASE,SLIP_ESC);
			UARTCharPut(UART1_BASE,SLIP_ESC_END);
			break;

		// if it's the same code as an ESC character,  we send a special two character code so as not to make the receiver think we sent an ESC
		case SLIP_ESC:
			UARTCharPut(UART1_BASE,SLIP_ESC);
			UARTCharPut(UART1_BASE,SLIP_ESC_ESC);
			break;

		// otherwise, we just send the character
		default:
			UARTCharPut(UART1_BASE,*data);
		}
		data++;
	}
	UARTCharPut(UART1_BASE,SLIP_END);
	xSemaphoreGive( xTxMutex );

	return temp;

}


//! funkce volan� p�i p�ijmu znaku
void UART1_IRQHandler( void)
{
	signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	char data;

#if configUSE_TRACE_FACILITY==1
	portENTER_CRITICAL();
	vTraceStoreISRBegin(1);
	portEXIT_CRITICAL();
#endif

	//! dokud buffer nen� pr�zdn�
	while( UARTCharsAvail(UART1_BASE) )
	{
		//p�idej data do fronty
		data = UARTCharGetNonBlocking(UART1_BASE);
		xQueueSendFromISR( xSerialReceive, &data, &xHigherPriorityTaskWoken );
	}

	//p�edej ��zen� OS
	if( xHigherPriorityTaskWoken != pdFALSE )
	{
			taskYIELD();
	}

	// nuluj p��znak p�eru�en�
	UARTIntClear(UART1_BASE,UART_INT_RX);
#if configUSE_TRACE_FACILITY==1
	portENTER_CRITICAL();
	vTraceStoreISREnd();
	portEXIT_CRITICAL();
#endif
}

//! funkce reprezentuj�c� FreeRTOS �lohu SLIP
void SlipSerial_task( void * param)
{
	static int received = 0;
	static int escBefore = 0;

	vTaskDelay(100);

	// nekone�n� smy�ka
	while(1)
	{
		char data;
		// p�i p�ijmu dat frontou zpracuj
		if( xQueueReceive(xSerialReceive, &data,100 ) == pdTRUE )
		{
			switch(data)
			{
			// zna�ka konce
			case  SLIP_END:
				if (received > 0)
				{
					//process packet callback
					SlipSerialProcessPacket(packet_buffer, received);
				}
				received = 0;
				break;

				// zna�ka speci�ln�ho znaku
			case SLIP_ESC:
				escBefore = 1;
				break;
				// speci�ln� znak m�sto END
			case SLIP_ESC_END:
				if (escBefore == 1)
				{
					if (received < MAX_PACKET_LENGH)
						packet_buffer[received++] = SLIP_END;
					escBefore = 0;
				}
				else
				{
					if (received < MAX_PACKET_LENGH)
						packet_buffer[received++] = data;
				}
				break;
				// speci�ln� znak m�sto ESC
			case SLIP_ESC_ESC:
				if (escBefore == true)
				{
					if (received < MAX_PACKET_LENGH)
						packet_buffer[received++] = SLIP_ESC;
					escBefore = 0;
				}
				else
				{
					if (received < MAX_PACKET_LENGH)
						packet_buffer[received++] = data;
				}
				break;
				// pro ostatn� znaky p�id�me
			default:
				if (received < MAX_PACKET_LENGH)
					packet_buffer[received++] = data;
				break;
			}

		} else
		{
			//communication timeout callback
			SlipSerialReceiveTimeout();
		}
	}

	//vTaskDelete( NULL );
}
