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

signed short AccelerometerRead( void);
short AccelerometerBandgapTest( void);
short AccelerometerPowerUp( void);

xSemaphoreHandle xRequestAccData;

//m�sto pro na�ten� zrychlen�
short int accData[2] = {0,0};


// inicializa�n� funkce
signed long  AccelerometerInit(  unsigned long priority)
{
	short temp;

	// Enables I2C0 and GPIO B
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	//Propojen�  I2C sign�l�
	GPIOPinTypeI2C(I2C_SCL_PORT, I2C_SCL);
	GPIOPinTypeI2C(I2C_SDA_PORT, I2C_SDA);

	// nastaven� re�imu a rychlosti
	I2CMasterInitExpClk(I2C_MASTER_BASE, SysCtlClockGet(), true);

	// zapnut� ACC
	temp = AccelerometerPowerUp();

	// NEED to be edited
	unsigned short i = 1;
	while(i != 0)
	{
		i++;
	}

	// bandgap ACC test
	temp = AccelerometerBandgapTest();

	if (temp < 9)
		return temp;


	// FreeRTOS objects
	vSemaphoreCreateBinary( xRequestAccData );
	if( xRequestAccData == NULL )
	{
		return pdFAIL;
	}

	return xTaskCreate(Accelerometer_task, (signed portCHAR *) "ACC", 256, NULL, priority , NULL);;
}

// FreeRTOS function
void Accelerometer_task( void * pvParameters )
{
	while(1)
	{
		// we wait for actulization request
		if ( xSemaphoreTake( xRequestAccData, 100 ) == pdPASS )
		{

		}
		// acceleration actualization
		if (AccelerometerRead()<0)
		{
			//error
		}

	}
	//vTaskDelete( NULL );
}

signed long AccelerometerRequestData( void)
{
	return xSemaphoreGive( xRequestAccData );
}

float AccelerometerGetX( void)
{
	return  (float)(accData[0]-2048)/400.0*9.81;
}

float AccelerometerGetY( void)
{
	return  (float)(accData[1]-2048)/400.0*9.81;
}

// reading a new data
signed short AccelerometerRead( void)
{
	unsigned long temp;
	signed short err;

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

// Bandgap test
short AccelerometerBandgapTest( void)
{
	// variables
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

	// delay
	unsigned short i = 1;
	while(i != 0)
	{
		i++;
	}

	// ACC dummy read
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

// acc. soft power supply enable
short AccelerometerPowerUp( void)
{
	// variables
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
