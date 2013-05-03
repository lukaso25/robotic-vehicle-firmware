#ifndef  __COMMON_DEFS__
#define  __COMMON_DEFS__


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
	double x;
	double y;
	double fi;
};

#endif//__COMMON_DEFS__
