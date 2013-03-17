#ifndef __REGULATOR_H__
#define __REGULATOR_H__

struct RegulatorParams
{
	//!
	short desired;

	//!
	short error;

	//!
	double sum;

	//! upravit s modifikací ochrany proti pøeintegrování
	unsigned short batt_voltage;

	//!
	float K;
	float Ti;
};


#endif//__REGULATOR_H__
