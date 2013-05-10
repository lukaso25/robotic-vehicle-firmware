#ifndef __REGULATOR_H__
#define __REGULATOR_H__

/*! \file Regulator.h
 * \author Lukas Otava
 * \date 2013
 * \defgroup Regulator Regulator module
 *
 * This module contain modification PID (PSD) algorithm based regulator with structure that hold actual values and parameters.
 *
 * There is basic version of PID (PSD) algorithm, that is selected by macro REG_VERSION 1.
 *
 * Another version called Beta-PSD structure (from prof. Petr Pivonka course) is selected by macro REG_VERSION 2.
 *  There are some other parameters N, Beta and Kip, that should be set-up by \function RegulatorSetParam().
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
	unsigned short limit;

	//! output gain correction
	float outputScale;

	//!
	float sum;

	//!
	float der;

	//!
	float saturationDiff;

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
	//! derivation exp coeficient
	float Dexp;
};


/*! \brief Available states of MotorControl module
 * \ingroup Regulator */
//#define REG_VERSION (2)

/*! \brief When this macro is set to 1, output of regulator will be in range <0;+limit>, otherwise <-limit,+limit>
 * \ingroup Regulator */
#define REG_UNIPOLAR_LIMIT (0)

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
short RegulatorAction(struct RegulatorParams * rp, short measurement, unsigned char manualMode);

/*! \brief Function providing setting of PID parameters
 *
 * 	Transfer function of regulator: \f$F_R(s) = K_r (1 + \frac{T_D * s}{1 + \epsilon s} + \frac{1}{T_I*s} )\f$
 *
 * \ingroup Regulator
 * \param rp pointer to the structure, that contain regulator data
 * \param Kr regulator gain
 * \param Ti inverted integration time constant (\f$T_i=T_{S}/T_I\f$ compared to continuous system)
 * \param Td derivation time constant (\f$T_d=T_D/T_{S}\f$  compared to continuous system)
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
 * \param N this parameter is filtration coefficient of derivation part of regulator (1.0 - 20.0), lower for higher noise in process
 *  */
void RegulatorSetParams(struct RegulatorParams * rp, float Beta, float Kip, float N);

/*! \brief Function providing setting of output symmetric limit
 * \ingroup Regulator
 * \param limit symmetric limit of saturation nonlinearity on the output of regulator
 *  */
void RegulatorSetScaleLimit(struct RegulatorParams * rp, float outputScale, unsigned short limit);


void RegulatorResetStates(struct RegulatorParams * rp);

#endif//__REGULATOR_H__
