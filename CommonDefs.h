#ifndef  __COMMON_DEFS__
#define  __COMMON_DEFS__

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_uart.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pwm.h"
#include "driverlib/qei.h"
#include "driverlib/i2c.h"
#include "driverlib/adc.h"

//! this macro convert value from ADC to number in volts (V)
#define ADC2VOLTAGE(x) ((x)*(9.617/1023.0))

//! this macro convert value from ADC to number in mili ampers (mA)
#define ADC2CURRENT(x) ((x)*(375.0/200.0))

//!this macro convert value in volts (V) into value from ADC
#define VOLTAGE2ADC(x) ((x)*(1023.0/9.617))

//!this macro convert value in mili ampers (mA) into value from ADC
#define CURRENT2ADC(x) ((x)*(200.0/375.0))

struct Position
{
	float x;
	float y;
	float theta;
};


//! HW definitions I2C


//#define I2C_PERIPHERAL (SYSCTL_PERIPH_I2C)

//! I2C SCL PIN
#define I2C_SCL				(GPIO_PIN_2)
#define I2C_SCL_PORT		(GPIO_PORTB_BASE)

//! I2C SDA PIN
#define I2C_SDA				(GPIO_PIN_3)
#define I2C_SDA_PORT		(GPIO_PORTB_BASE)


//! HW definitions MOTORCONTROL

#define BRIDGE0_EN				(GPIO_PIN_7)
#define BRIDGE0_EN_PORT			(GPIO_PORTD_BASE)
#define BRIDGE0_IN1				(GPIO_PIN_0)
#define BRIDGE0_IN1_PORT		(GPIO_PORTF_BASE)
#define BRIDGE0_IN2				(GPIO_PIN_1)
#define BRIDGE0_IN2_PORT		(GPIO_PORTG_BASE)
#define BRIDGE0_FS				(GPIO_PIN_4)
#define BRIDGE0_FS_PORT			(GPIO_PORTB_BASE)

#define BRIDGE1_EN				(GPIO_PIN_1)
#define BRIDGE1_EN_PORT			(GPIO_PORTF_BASE)
#define BRIDGE1_IN1				(GPIO_PIN_0)
#define BRIDGE1_IN1_PORT		(GPIO_PORTB_BASE)
#define BRIDGE1_IN2				(GPIO_PIN_1)
#define BRIDGE1_IN2_PORT		(GPIO_PORTB_BASE)
#define BRIDGE1_FS				(GPIO_PIN_5)
#define BRIDGE1_FS_PORT			(GPIO_PORTB_BASE)

#define MOTOR_SHIFTER_OE		(GPIO_PIN_7)
#define MOTOR_SHIFTER_OE_PORT	(GPIO_PORTA_BASE)

#define QEI0_PORT	(GPIO_PORTC_BASE)
#define QEI0_PHA	(GPIO_PIN_4)
#define QEI0_PHB	(GPIO_PIN_6)

#define QEI1_PORT	(GPIO_PORTE_BASE)
#define QEI1_PHA	(GPIO_PIN_3)
#define QEI1_PHB	(GPIO_PIN_2)


//! USER SWITCH MACROS

#define USER_SWITCH_PERIPHERALS (SYSCTL_PERIPH_GPIOE|SYSCTL_PERIPH_GPIOB)
#define USER_SWITCH1_PORT	(GPIO_PORTE_BASE)
#define USER_SWITCH2_PORT	(GPIO_PORTE_BASE)
#define USER_SWITCH3_PORT	(GPIO_PORTB_BASE)
#define USER_SWITCH1		(GPIO_PIN_1)
#define USER_SWITCH2		(GPIO_PIN_0)
#define USER_SWITCH3		(GPIO_PIN_6)

#define USER_SWITCH_INIT() {SysCtlPeripheralEnable(USER_SWITCH_PERIPHERALS); \
							GPIOPinTypeGPIOInput(USER_SWITCH1_PORT,USER_SWITCH1);\
							GPIOPadConfigSet(USER_SWITCH1_PORT,USER_SWITCH1,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);\
							GPIOPinTypeGPIOInput(USER_SWITCH1_PORT,USER_SWITCH2);\
							GPIOPadConfigSet(USER_SWITCH2_PORT,USER_SWITCH2,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);\
							GPIOPinTypeGPIOInput(USER_SWITCH1_PORT,USER_SWITCH3);\
							GPIOPadConfigSet(USER_SWITCH3_PORT,USER_SWITCH3,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);}

#define USER_SWITCH_READ1() ( GPIOPinRead(USER_SWITCH1_PORT,USER_SWITCH1)==0 )
#define USER_SWITCH_READ2() ( GPIOPinRead(USER_SWITCH2_PORT,USER_SWITCH2)==0 )
#define USER_SWITCH_READ3() ( GPIOPinRead(USER_SWITCH3_PORT,USER_SWITCH3)==0 )


#endif//__COMMON_DEFS__
