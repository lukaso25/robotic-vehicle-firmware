
#include "CANtest.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/can.h"

#include "StatusLED.h"

void CANtest_task( void * param)
{
	// buffers
	tCANMsgObject sMsgObjectRemoteTx;
	tCANMsgObject sMsgObjectRx;
	tCANMsgObject sMsgObjectTx;
	unsigned char ucBufferRemoteOut[8]  = "0123REM\0";
	unsigned char ucBufferIn[4];
	unsigned char ucBufferOut[8] = "\0\0\0\0\0\0\0\0";

	//IO init
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);

	// propojen� jednotky s GPIO piny
	GPIOPinTypeCAN(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// za��tek inicializace
	CANInit(CAN0_BASE);

	// konfigurace rychlostip�enosu 1 Mbit p�i 8 Mhz
	CANBitRateSet( CAN0_BASE, 8000000, 1000000);

	// Povolen� CAN0, ukon�en� inicializace
	CANEnable(CAN0_BASE);

	//konfigurace p�ij�mac�ho objektu
	sMsgObjectRx.ulMsgID = 0x04;
	sMsgObjectRx.ulMsgIDMask = 0;
	sMsgObjectRx.ulFlags = 0;
	sMsgObjectRx.ulMsgLen = 4;
	sMsgObjectRx.pucMsgData = ucBufferIn;
	CANMessageSet(CAN0_BASE, 4, &sMsgObjectRx, MSG_OBJ_TYPE_RX);


	// Konfigurace odes�lan�ho remote objektu
	sMsgObjectRemoteTx.ulMsgID = 0x03;
	sMsgObjectRemoteTx.ulFlags = 0;
	sMsgObjectRemoteTx.ulMsgLen = 8;
	sMsgObjectRemoteTx.pucMsgData = ucBufferRemoteOut;
	CANMessageSet(CAN0_BASE, 3, &sMsgObjectRemoteTx, MSG_OBJ_TYPE_RXTX_REMOTE);

	// Konfigurace odes�lan�ho objektu
	sMsgObjectTx.ulMsgID = 0x02;
	sMsgObjectTx.ulFlags = 0;
	sMsgObjectTx.ulMsgLen = 8;
	sMsgObjectTx.pucMsgData = ucBufferOut;
	CANMessageSet(CAN0_BASE, 2, &sMsgObjectTx, MSG_OBJ_TYPE_TX);

	vTaskDelay(100);

	// nekone�n� smy�ka s aktivn�m �ek�n�m na p��choz� zpr�vu s n�sledn�m odesl�n�m
	while(1)
	{
		vTaskDelay(1);
		while((CANStatusGet(CAN0_BASE, CAN_STS_NEWDAT) & (3<<3)) == 0)
		{
			vTaskDelay(1);
		}
		// Read the message out of the message object.
		CANMessageGet(CAN0_BASE, 4, &sMsgObjectRx, true);

		ucBufferOut[0]++;
		ucBufferOut[1] = ucBufferIn[0];
		CANMessageSet(CAN0_BASE, 2, &sMsgObjectTx, MSG_OBJ_TYPE_TX);
	}
}
