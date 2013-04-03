#ifndef __REGULATOR_H__
#define __REGULATOR_H__

/*! \file Regulator.h
 * \author Lukas Otava
 * \date 2013
 * \defgroup Regulator Regulator module
 *
 * This module contain modification PID algorithm based regulator with structure that hold actual values and parameters.
 *
 * */

#define SUM_LIMIT (1000)

/*! \brief Available states of MotorControl module
 * \ingroup Regulator */
struct RegulatorParams
{
	//! actual value measured by output sensor
	short actual;

	//! unlimited output value from regulator
	short action;

	//! error signal as input into the regulator
	short error;

	//! desired value setpoint
	short desired;

	//! output limit value
	short limit;
	// value for manual output
	//short manual;

	//!
	float sum;
	//!
	short saturationDiff;

	//! regulator gain
	float K;
	//! integration constant
	float Ti;
	//! derivation value
	float Td;
	//! beta parametr (0-1)
	float Beta;
	//! wind-up protection
	float Kip;

	//! actual battery voltage
	unsigned short batt_voltage;
};

/*! \brief Available states of MotorControl module
 * \ingroup Regulator */
#define REG_VERSION (1)

/*! \brief Available states of MotorControl module
 *
 * 	This function compute action value by modified PID control algorithm.
 *
 * 	Transfer function of regulator:
 *
 * \ingroup Regulator
 * \param rp pointer to the structure, that contain regulator data
 * \param measurement measured value by sensor on the output of controlled system
 * \return action value to the controlled system
 *  */
short SpeedReg(struct RegulatorParams * rp, short measurement);

#endif//__REGULATOR_H__
