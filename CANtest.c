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

signed long CANtestInit( unsigned long priority)
{
	return xTaskCreate(CANtest_task, (signed portCHAR *) "CAN", 256, NULL, priority, NULL);
}

void CANtest_task( void * param)
{
	// message objects declaration
	tCANMsgObject sMsgObjectRemoteTx;
	tCANMsgObject sMsgObjectRx;
	tCANMsgObject sMsgObjectTx;

	// message buffers initialization
	unsigned char ucBufferRemoteOut[8]  = "0123REM\0";
	unsigned char ucBufferIn[4];
	unsigned char ucBufferOut[8] = "\0\0\0\0\0\0\0\0";

	//clock & IO initialization
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);

	// GPIO and CAN module connection
	GPIOPinTypeCAN(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// CAN initialization beginning
	CANInit(CAN0_BASE);

	// Peripheral clock and desired speed configuration
	CANBitRateSet( CAN0_BASE, 8000000, 1000000);

	// CAN peripheral prepared for operation, we can enable it
	CANEnable(CAN0_BASE);

	// RX message object configuration id 4, length 4
	sMsgObjectRx.ulMsgID = 0x04;
	sMsgObjectRx.ulMsgIDMask = 0;
	sMsgObjectRx.ulFlags = 0;
	sMsgObjectRx.ulMsgLen = 4;
	sMsgObjectRx.pucMsgData = ucBufferIn;
	// this starts reception
	CANMessageSet(CAN0_BASE, 4, &sMsgObjectRx, MSG_OBJ_TYPE_RX);

	// RXTX remote object configuration id 3 length 8
	sMsgObjectRemoteTx.ulMsgID = 0x03;
	sMsgObjectRemoteTx.ulFlags = 0;
	sMsgObjectRemoteTx.ulMsgLen = 8;
	sMsgObjectRemoteTx.pucMsgData = ucBufferRemoteOut;
	// this enable automatic response for remote request id 3, this is done in background
	CANMessageSet(CAN0_BASE, 3, &sMsgObjectRemoteTx, MSG_OBJ_TYPE_RXTX_REMOTE);

	// TX object configuration id 2
	sMsgObjectTx.ulMsgID = 0x02;
	sMsgObjectTx.ulFlags = 0;
	sMsgObjectTx.ulMsgLen = 8;
	sMsgObjectTx.pucMsgData = ucBufferOut;
	// This sends CAN TX object
	CANMessageSet(CAN0_BASE, 2, &sMsgObjectTx, MSG_OBJ_TYPE_TX);

	//dummy delay
	vTaskDelay(100);

	// never ending loop handling CAN message reception and transmission modified packets
	while(1)
	{
		//waiting for new message int object according to mask
		while((CANStatusGet(CAN0_BASE, CAN_STS_NEWDAT) & (3<<3)) == 0)
		{
			//dummy delay - we don't require best response time - this decrease CPU load
			vTaskDelay(100);
		}
		// Read the message out of the message object
		CANMessageGet(CAN0_BASE, 4, &sMsgObjectRx, true);

		// modify transmit buffer
		ucBufferOut[0]++;
		ucBufferOut[1] = ucBufferIn[0];

		// send CAN message with modified data
		CANMessageSet(CAN0_BASE, 2, &sMsgObjectTx, MSG_OBJ_TYPE_TX);
	}
}
