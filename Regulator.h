#ifndef __REGULATOR_H__
#define __REGULATOR_H__

/*! \file Regulator.h
 * \author Lukas Otava
 * \date 2013
 * \defgroup Regulator Regulator module
 *
 * This module contain modified PID (PSD) algorithm based regulator called Beta-PSD structure (from prof. Petr Pivonka course). This kind of regulator has roots in Takahashi PID modification.
 * All state variables and parameters are stored in RegulatorParams structure.
 *
 * Example usage:
   \code{c}
  	struct RegulatorParams REG;

	RegulatorResetStates(&reg);

	RegulatorSetPID(&reg, 0.5, 0.19, 1.3);

	RegulatorSetParams(&reg, 0.9, 0.5, 1.0);

	RegulatorSetScaleLimit(&reg, (pwm_period/MOTOR_PULSES_PER_VOLT/8.0), pwm_period);

	reg.desire = 100;

	while (1)
	{
		waitNewData();
		action_value = RegulatorAction(&reg, measured_value, 0);
	}
	\endcode
 *
 * State diagram of the modified version of PSD regulator:
 *
 * \image html regulator.png
 *
 * Another version called Beta-PSD structure (from prof. Petr Pivonka course) is selected by macro REG_VERSION 2.
 *  There are some other parameters N, Beta and Kip, that should be set-up by  RegulatorSetParam().
 *
 * */

/*!
 * \brief This macro enable only PI regulator functionality
 * \ingroup Regulator */
#define REGULATOR_PI_VERSION_ONLY	(1)

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

	//! regulator state variable
	float sum;

	//! regulator state variable
	float der;

	//! dynamic anti-wind-up feedback variable
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

/*! \brief When this macro is set to 1, output of regulator will be in range <0;+limit>, otherwise <-limit,+limit>
 * \ingroup Regulator */
#define REG_UNIPOLAR_LIMIT (0)

/*! \brief Function that regulator action value according to measurement
 *
 * 	This function compute action value by modified PID control algorithm.
 *
 * 	Transfer function of regulator:
 *
 * 	\f$
 * 	U(z) = K_R \left[ \beta W(z) - Y(z) + \frac{T_S z^{-1}}{T_I (1 - z^{-1})}(W(z) - Y(z)) - N \frac{1 - z^{-1}}{1 - e^{-\frac{T_S N}{T_D}} z^{-1}} Y(z) \right]
 * 	\f$
 *
 * 	Where U is action value,
 * 	W is desired value and
 * 	Y measured value.
 *
 * \ingroup Regulator
 * \param rp Pointer to the structure, that contain regulator data
 * \param measurement Measured value by sensor on the output of controlled system
 * \param manualMode This parameter switch from closed-loop control into open-loop control
 * \return action value to the controlled system
 *  */
short RegulatorAction(struct RegulatorParams * rp, short measurement, unsigned char manualMode);

/*! \brief Function providing setting of PID parameters
 *
 *
 * \ingroup Regulator
 * \param rp Pointer to the regulator structure.
 * \param Kr Regulator proportional gain.
 * \param Ti Inverted integration time constant (\f$T_i=T_{S}/T_I\f$ compared to continuous system)
 * \param Td Derivation time constant (\f$T_d=T_D/T_{S}\f$ compared to continuous system)
 *  */
void RegulatorSetPID(struct RegulatorParams * rp, float Kr, float Ti, float Td);

/*! \brief Function providing setting of regulator parameters
 *
 * This parameters are additional for modified version, the Beta-PSD regulator.
 * There is difference in measured and desired value routing on input of regulator.
 *
 * \ingroup Regulator
 * \param rp Pointer to the regulator structure.
 * \param Beta Ratio of proportional gain of desired value (0.0 - 1.0).
 * \param Kip This parameter is proportional gain used in dynamic anti wind-up feedback loop.
 * \param N this parameter is filtration coefficient of derivation part of regulator (1.0 - 20.0), lower for higher noise in process.
 *  */
void RegulatorSetParams(struct RegulatorParams * rp, float Beta, float Kip, float N);

/*! \brief Function providing setting of output symmetric limit
 * \ingroup Regulator
 * \param rp Pointer to the regulator structure.
 * \param outputScale Regulator make-up gain situated in front of the output nonlinearity model.
 * \param limit Limit value of the saturation nonlinearity on the output of regulator.
 *  */
void RegulatorSetScaleLimit(struct RegulatorParams * rp, float outputScale, unsigned short limit);

/*! \brief Function providing initialization state variable to zeros
 * \ingroup Regulator
 * \param rp Poniter to the regulator structure.
 *  */
void RegulatorResetStates(struct RegulatorParams * rp);

/*! \brief Function providing setting of desired value
 * \ingroup Regulator
 * \param rp Pointer to the regulator structure.
 * \param desiredValue New desired value.
 *  */
void RegulatorSetDesired(struct  RegulatorParams * rp, short desiredValue);

#endif//__REGULATOR_H__
