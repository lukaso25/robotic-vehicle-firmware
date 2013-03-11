
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
	//tCANMsgObject sMsgObjectRx;
	tCANMsgObject sMsgObjectTx;
	//unsigned char ucBufferIn[8];
	unsigned char ucBufferOut[8] = "AhojCAN\0";

	//IO init
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);

	// propojení jednotky s GPIO piny
	GPIOPinTypeCAN(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// zaèátek inicializace
	CANInit(CAN0_BASE);

	// konfigurace rychlostipøenosu 1 Mbit pøi 8 Mhz
	CANBitRateSet( CAN0_BASE, 8000000, 1000000);

	// Povolení CAN0, ukonèení inicializace
	CANEnable(CAN0_BASE);

	// Konfigurace odesílaného objektu
	sMsgObjectTx.ulMsgID = 0x401;
	sMsgObjectTx.ulFlags = 0;
	sMsgObjectTx.ulMsgLen = 8;
	sMsgObjectTx.pucMsgData = ucBufferOut;
	CANMessageSet(CAN0_BASE, 2, &sMsgObjectTx, MSG_OBJ_TYPE_TX);

	vTaskDelay(100);

	// prázdná nekoneèná smyèka s aktivním èekáním
	while(1)
	{
		vTaskDelay(1000);
	}
}
