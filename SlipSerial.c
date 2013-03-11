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


// SLIP makra znakù
#define SLIP_END             0xC0    /* indicates end of packet */
#define SLIP_ESC             0xDB    /* indicates byte stuffing */
#define SLIP_ESC_END         0xDC    /* ESC ESC_END means END data byte */
#define SLIP_ESC_ESC         0xDD    /* ESC ESC_ESC means ESC data byte */

// maximální delka pøíjmaného paketu
#define MAX_PACKET_LENGH	(300)


// objekty OS
xSemaphoreHandle xTxMutex;
char packet_buffer[MAX_PACKET_LENGH];
static xQueueHandle xSerialReceive;

//! inicializaèní funkce
int SlipSerialInit(unsigned long int baudrate)
{
	//! povolení hodin UART and GPIO module
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);


	//! propojení vývodù s jednotkou
	GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_2 | GPIO_PIN_3);

	//! Inicializace formátu a rychlosti
	UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 57600,
			(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

	// Povolení pøerušení a jeho nastavení priority
	UARTIntEnable(UART1_BASE, UART_INT_RX);

	IntPrioritySet(INT_UART1,(6<<5));
	IntEnable(INT_UART1);

	xSerialReceive = xQueueCreate( 32, ( unsigned portBASE_TYPE ) sizeof( char ) );

	//! Povolení UART.
	UARTEnable(UART1_BASE);

	//! MUTEX ONLY FROM THREAD!!!
	xTxMutex = xSemaphoreCreateMutex();
	//vSemaphoreCreateBinary(xTxMutex); //- from interrupt


	if (xTxMutex != NULL)
	{
		return 0; // sucess
	}
	else
	{
		return -1;// fail
	}
}

//! odeslání ukonèovacího znaku
void SlipEnd( void)
{
	UARTCharPut(UART1_BASE,SLIP_END);
}

//! odeslání bufferu jako slip paket
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


//! funkce volaná pøi pøijmu znaku
void UART1_IRQHandler( void)
{
	signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	char data;

#if configUSE_TRACE_FACILITY==1
	portENTER_CRITICAL();
	vTraceStoreISRBegin(1);
	portEXIT_CRITICAL();
#endif

	//! dokud buffer není prázdný
	while( UARTCharsAvail(UART1_BASE) )
	{
		//pøidej data do fronty
		data = UARTCharGetNonBlocking(UART1_BASE);
		xQueueSendFromISR( xSerialReceive, &data, &xHigherPriorityTaskWoken );
	}

	//pøedej øízení OS
	if( xHigherPriorityTaskWoken != pdFALSE )
	{
			taskYIELD();
	}

	// nuluj pøíznak pøerušení
	UARTIntClear(UART1_BASE,UART_INT_RX);
#if configUSE_TRACE_FACILITY==1
	portENTER_CRITICAL();
	vTraceStoreISREnd();
	portEXIT_CRITICAL();
#endif
}

//! režim motoru
char motorMode = MOTOR_RUNNING;

//! funkce reprezentující FreeRTOS úlohu SLIP
void SlipSerial_task( void * param)
{
	static int received = 0;
	static int escBefore = 0;

	vTaskDelay(100);

	// nekoneèná smyèka
	while(1)
	{
		char data;
		// pøi pøijmu dat frontou zpracuj
		if( xQueueReceive(xSerialReceive, &data,100 ) == pdTRUE )
		{
			switch(data)
			{
			// znaèka konce
			case  SLIP_END:
				if (received > 0)
				{
					switch (packet_buffer[0])
					{
					case ID_MOTOR_ACT:
						//máme pøijato :-)
						if (received<5)
						{
							//poškozená data
						}
						else
						{
							//need to be tested
						}
						StatusLEDClearError(ERROR_COMM_SLIP);
						myDrive.mot1.speed_des = packet_buffer[1]|(packet_buffer[2]<<8);
						myDrive.mot2.speed_des = packet_buffer[3]|(packet_buffer[4]<<8);
						MotorControlSetState(motorMode);
						break;
					case ID_MOTOR_MODE:
						if (received<2)
						{

						}
						motorMode = packet_buffer[1];
						MotorControlSetState(motorMode);
						break;
					case ID_REG_PARAMS:
						if (received<6)
						{

						}
						memcpy(&myDrive.K,&packet_buffer[1],8);
						break;
					}
				}
				received = 0;
				break;

				// znaèka speciálního znaku
			case SLIP_ESC:
				escBefore = 1;
				break;
				// speciální znak místo END
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
				// speciální znak místo ESC
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
				// pro ostatní znaky pøidáme
			default:
				if (received < MAX_PACKET_LENGH)
					packet_buffer[received++] = data;
				break;
			}

		} else
		{
			MotorControlSetState(MOTOR_SHUTDOWN);
			StatusLEDSetError(ERROR_COMM_SLIP);
			//communication timeout
		}
	}

	//vTaskDelete( NULL );
}
