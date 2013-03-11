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


#define I2C_SCL				(GPIO_PIN_2)
#define I2C_SCL_PORT		(GPIO_PORTB_BASE)

#define I2C_SDA				(GPIO_PIN_3)
#define I2C_SDA_PORT		(GPIO_PORTB_BASE)

//adresa akcelerometru
#define ACC_I2C_ADR		(0x20>>1)

//m�sto pro na�ten� zrychlen�
short int accs[2] = {0,0};

// na�ten� a ulo�en� hodnot zrychlen� z akcelerometru
short AccelerometerRead( void)
{
	unsigned long temp;
	short err = 0;

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
	accs[0] = 0;
	accs[0] = I2CMasterDataGet(I2C_MASTER_BASE)<<8;
	if((temp = I2CMasterErr(I2C_MASTER_BASE)) != 0)
	{
		err = -1;
		//error happened
	}

	// Initiate send of character from Master to Slave
	I2CMasterControl(I2C_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
	// Delay until transmission completes
	while(I2CMasterBusy(I2C_MASTER_BASE)){};
	accs[0] |= I2CMasterDataGet(I2C_MASTER_BASE)&0xFF;
	if((temp = I2CMasterErr(I2C_MASTER_BASE)) != 0)
	{
		err = -1;
		//error happened
	}

	// Initiate send of character from Master to Slave
	I2CMasterControl(I2C_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
	// Delay until transmission completes
	while(I2CMasterBusy(I2C_MASTER_BASE)){};
	accs[1] = 0;
	accs[1] = I2CMasterDataGet(I2C_MASTER_BASE)<<8;
	if((temp = I2CMasterErr(I2C_MASTER_BASE)) != 0)
	{
		err = -1;
		//error happened
	}

	I2CMasterControl(I2C_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
	// Delay until transmission completes
	while(I2CMasterBusy(I2C_MASTER_BASE)){};
	accs[1] |= I2CMasterDataGet(I2C_MASTER_BASE)&0xFF;
	if((temp = I2CMasterErr(I2C_MASTER_BASE)) != 0)
	{
		err = -1;
		//error happened
	}

	return err;
}

//nulov�n� akcelerometry, tepeln� kompenzace
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

	//! n�kolik �ten� pro z�sk�n� pr�m�ru
	vTaskDelay(20);
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

// softwarov� povolen� nap�jen� akcelerometru
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


// inicializa�n� funkce
short AccelerometrInit( void)
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

	vTaskDelay(100);

	// nulov�n� ACC
	temp = AccelerometerBandgapTest();

	return temp;
}

// funkce reprezentuj�c� FreeRTOS �lohu
void Accelerometer_task( void * pvParameters )
{

	while(1)
	{
		//na�ten�
		if (AccelerometerRead()<0)
		{
			//error
		}

		//odesl�n� a �ek�n�
		SlipSend(ID_ACC_STRUCT,(char *) &accs,2*sizeof(short int));
		vTaskDelay(20);
	}
	//vTaskDelete( NULL );

}
