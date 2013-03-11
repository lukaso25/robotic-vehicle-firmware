#ifndef  __SLIPSERIAL_H__
#define  __SLIPSERIAL_H__

int SlipSerialInit(unsigned long int baudrate);
void SlipEnd( void);
int SlipSend(char id, char * data, int len);
void UART1_IRQHandler( void);
void SlipSerial_task( void * param);

enum slip_ids{
	ID_ACC_STRUCT = 0x10,
	ID_REG = 0x11,
	ID_TASKLIST = 0x12,
	ID_ADC = 0x13,
	ID_MOTOR_ACT = 0x14,
	ID_MOTOR_MODE = 0x15,
	ID_TIME_STAMP = 0x16,
	ID_REG_PARAMS = 0x17
};

#endif//__SLIPSERIAL_H__
