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

/*! \brief Available states of MotorControl module
 * \ingroup Regulator */
struct RegulatorParams
{
	//! actual value measured by output sensor
	short measured;

	//! unlimited output value from regulator
	short action;

	//! error signal as input into the regulator
	short error;

	//! desired value set-point
	short desired;

	//! output limit value
	short limit;
	// value for manual output
	//short manual;

	//!
	float sum;

	//!
	float der;
	//!
	short saturationDiff;

	//! regulator gain
	float Kr;
	//! integration constant
	float Ti;
	//! derivation value
	float Td;
	//! beta parameter (0-1)
	float Beta;
	//! wind-up protection feedback gain
	float Kip;
	//! the derivation filtration coefficient
	float N;

	//! actual battery voltage
	unsigned short batt_voltage;
};

/*! \brief Available states of MotorControl module
 * \ingroup Regulator */
#define REG_VERSION (2)

/*! \brief Function that regulator action value according to measurement
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
short RegulatorAction(struct RegulatorParams * rp, short measurement);

/*! \brief Function providing setting of PID parameters
 *
 * 	Transfer function of regulator: \f$F_R(s) = K_r (1 + T_D *\frac{s}{1 + \epsilon s} + \frac{1}{T_I*s} )\f$
 *
 * \ingroup Regulator
 * \param rp pointer to the structure, that contain regulator data
 * \param Kr regulator gain
 * \param Ti integration time constant (\f$T_i=T_I/T_VZ\f$ compared to continuous system)
 * \param Td derivation time constant (\f$T_d=T_VZ/T_D\f$  compared to continuous system)
 *  */
void RegulatorSetPID(struct RegulatorParams * rp, float Kr, float Ti, float Td);

/*! \brief Function providing setting of regulator parameters
 *
 * This parameters are additional for modified version, the Beta-PSD regulator.
 * There is difference in measured and desired value routing on input of regulator.
 *
 * \ingroup Regulator
 * \param rp pointer to the structure, that contain regulator data
 * \param Beta ratio of proportional gain of desired value (0.0 - 1.0)
 * \param Kip this parameter is proportional gain user in anti wind-up feedback loop
 * \param N this parameter is filtration coefficient of derivation part of regulator (3.0 - 10.0)
 *  */
void RegulatorSetParam(struct RegulatorParams * rp, float Beta, float Kip, float N);

/*! \brief Function providing setting of output symmetric limit
 * \ingroup Regulator
 * \param limit symmetric limit of saturation nonlinearity on the output of regulator
 *  */
void RegulatorSetLimit(struct RegulatorParams * rp, short limit);

#endif//__REGULATOR_H__
