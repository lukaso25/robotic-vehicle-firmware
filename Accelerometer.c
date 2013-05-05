#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "timers.h"

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/i2c.h"
#include "driverlib/gpio.h"

#include "StatusLED.h"
#include "Accelerometer.h"
#include "SlipSerial.h"


#define ACC_I2C_ADR		(0x20>>1)


xSemaphoreHandle xRequestAccData;

//místo pro naètená zrychlení
short int accData[2] = {0,0};

// naètení a uložení hodnot zrychlení z akcelerometru
signed short AccelerometerRead( void)
{
	unsigned long temp;
	signed short err;

	//! JE TØEBA OPRAVIT ERROR HANDLING!!!

	// Specify slave address
	I2CMasterSlaveAddrSet(I2C_MASTER_BASE, ACC_I2C_ADR, false);

	I2CMasterDataPut(I2C_MASTER_BASE, 0x01);
	// Initiate send of character from Master to Slave
	I2CMasterControl(I2C_MASTER_BASE, I2C_MASTER_CMD_SINGLE_SEND);
	// Delay until transmission completes
	while(I2CMasterBusy(I2C_MASTER_BASE)){};
	if((temp = I2CMasterErr(I2C_MASTER_BASE)) != 0)
	{
		err = -1;
		//error happened
	}

	I2CMasterSlaveAddrSet(I2C_MASTER_BASE, ACC_I2C_ADR, true);

	// Initiate send of character from Master to Slave
	I2CMasterControl(I2C_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
	// Delay until transmission completes
	while(I2CMasterBusy(I2C_MASTER_BASE)){};
	accData[0] = 0;
	accData[0] = I2CMasterDataGet(I2C_MASTER_BASE)<<8;
	if((temp = I2CMasterErr(I2C_MASTER_BASE)) != 0)
	{
		err = -1;
		//error happened
	}

	// Initiate send of character from Master to Slave
	I2CMasterControl(I2C_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
	// Delay until transmission completes
	while(I2CMasterBusy(I2C_MASTER_BASE)){};
	accData[0] |= I2CMasterDataGet(I2C_MASTER_BASE)&0xFF;
	if((temp = I2CMasterErr(I2C_MASTER_BASE)) != 0)
	{
		err = -1;
		//error happened
	}

	// Initiate send of character from Master to Slave
	I2CMasterControl(I2C_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
	// Delay until transmission completes
	while(I2CMasterBusy(I2C_MASTER_BASE)){};
	accData[1] = 0;
	accData[1] = I2CMasterDataGet(I2C_MASTER_BASE)<<8;
	if((temp = I2CMasterErr(I2C_MASTER_BASE)) != 0)
	{
		err = -1;
		//error happened
	}

	I2CMasterControl(I2C_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
	// Delay until transmission completes
	while(I2CMasterBusy(I2C_MASTER_BASE)){};
	accData[1] |= I2CMasterDataGet(I2C_MASTER_BASE)&0xFF;
	if((temp = I2CMasterErr(I2C_MASTER_BASE)) != 0)
	{
		err = -1;
		//error happened
	}

	return err;


}

//nulování akcelerometry, tepelná kompenzace
short AccelerometerBandgapTest( void)
{
	unsigned long temp;
	short err = 0;
	// Specify slave address
	I2CMasterSlaveAddrSet(I2C_MASTER_BASE, ACC_I2C_ADR, false);

	// Initiate send of character from Master to Slave
	I2CMasterDataPut(I2C_MASTER_BASE, 0x00); // reg adr 0
	I2CMasterControl(I2C_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_START);

	// Delay until transmission completes
	while(I2CMasterBusy(I2C_MASTER_BASE)){};
	if((temp = I2CMasterErr(I2C_MASTER_BASE)) != 0)
	{
		err = -1;
		//error happened
	}

	I2CMasterDataPut(I2C_MASTER_BASE, (1<<2)); // value 0
	I2CMasterControl(I2C_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
	// Delay until transmission completes
	while(I2CMasterBusy(I2C_MASTER_BASE)){};
	if((temp = I2CMasterErr(I2C_MASTER_BASE)) != 0)
	{
		err = -1;
		//error happened
	}

	// NEED to be edited
	//! nìkolik ètení pro získání prùmìru
	unsigned short i = 1;
	while(i != 0)
	{
		i++;
	}
	err = AccelerometerRead();
	err = AccelerometerRead();
	err = AccelerometerRead();


	// Specify slave address
	I2CMasterSlaveAddrSet(I2C_MASTER_BASE, ACC_I2C_ADR, false);

	// Initiate send of character from Master to Slave
	I2CMasterDataPut(I2C_MASTER_BASE, 0x00); // reg adr 0
	I2CMasterControl(I2C_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_START);

	// Delay until transmission completes
	while(I2CMasterBusy(I2C_MASTER_BASE)){};
	if((temp = I2CMasterErr(I2C_MASTER_BASE)) != 0)
	{
		err = -1;
		//error happened
	}

	I2CMasterDataPut(I2C_MASTER_BASE, (0)); // value 0
	I2CMasterControl(I2C_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
	// Delay until transmission completes
	while(I2CMasterBusy(I2C_MASTER_BASE)){};
	if((temp = I2CMasterErr(I2C_MASTER_BASE)) != 0)
	{
		err = -1;
		//error happened
	}

	return err;
}

// softwarové povolení napájení akcelerometru
short AccelerometerPowerUp( void)
{
	unsigned long temp;
	short err = 0;
	// Specify slave address
	I2CMasterSlaveAddrSet(I2C_MASTER_BASE, ACC_I2C_ADR, false);

	// Initiate send of character from Master to Slave
	I2CMasterDataPut(I2C_MASTER_BASE, 0x00); // reg adr 0
	I2CMasterControl(I2C_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_START);

	// Delay until transmission completes
	while(I2CMasterBusy(I2C_MASTER_BASE)){};
	if((temp = I2CMasterErr(I2C_MASTER_BASE)) != 0)
	{
		//error happened
		switch (temp)
		{
		case I2C_MASTER_ERR_ADDR_ACK:
			err = -1;
			break;
		case I2C_MASTER_ERR_DATA_ACK:
		case I2C_MASTER_ERR_ARB_LOST:
		default:
			err = -2;
			break;
		}
	}

	I2CMasterDataPut(I2C_MASTER_BASE, 0x00); // value 0
	I2CMasterControl(I2C_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
	// Delay until transmission completes
	while(I2CMasterBusy(I2C_MASTER_BASE)){};
	if((temp = I2CMasterErr(I2C_MASTER_BASE)) != 0)
	{
		//error happened
		switch (temp)
		{

		case I2C_MASTER_ERR_DATA_ACK:
			err = -1;
			break;
		case I2C_MASTER_ERR_ARB_LOST:
		case I2C_MASTER_ERR_ADDR_ACK:
		default:
			err = -2;
			break;
		}
	}
	return err;
}


// inicializaèní funkce
signed portBASE_TYPE  AccelerometerInit(  unsigned portBASE_TYPE priority)
{
	short temp;

	// Enables I2C0 and GPIO B
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	//Propojení  I2C signálù
	GPIOPinTypeI2C(I2C_SCL_PORT, I2C_SCL);
	GPIOPinTypeI2C(I2C_SDA_PORT, I2C_SDA);

	// nastavení režimu a rychlosti
	I2CMasterInitExpClk(I2C_MASTER_BASE, SysCtlClockGet(), true);

	// zapnutí ACC
	temp = AccelerometerPowerUp();

	// NEED to be edited
	unsigned short i = 1;
	while(i != 0)
	{
		i++;
	}

	// nulování ACC
	temp = AccelerometerBandgapTest();

	// opravit logování chyb !!!
	if (temp < 9)
		return temp;

	vSemaphoreCreateBinary( xRequestAccData );
	if( xRequestAccData == NULL )
	{
		return pdFAIL;
	}

	return xTaskCreate(Accelerometer_task, (signed portCHAR *) "ACC", 256, NULL, priority , NULL);;
}

// funkce reprezentující FreeRTOS úlohu
void Accelerometer_task( void * pvParameters )
{
	while(1)
	{
		//naètení
		if ( xSemaphoreTake( xRequestAccData, 100 ) == pdPASS )
		{

		}
		if (AccelerometerRead()<0)
		{
			//error
		}

		//odeslání a èekání
	}
	//vTaskDelete( NULL );
}

signed portBASE_TYPE AccelerometerRequestData( void)
{
	// We would expect this call to fail because we cannot give
	// a semaphore without first "taking" it!
	return xSemaphoreGive( xRequestAccData );
}
