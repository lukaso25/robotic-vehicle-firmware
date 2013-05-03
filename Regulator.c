#include "Regulator.h"

#include <math.h>

short RegulatorAction(struct RegulatorParams * rp, short measurement, unsigned char manualMode)
{
	float flOut;
	short siOut;

	rp->measured = measurement;
	// error value
	rp->error = rp->desired - measurement;

	// PI version of regulator
#if REG_VERSION == 1
	// teste
	//regulator output
	flOut = rp->sum + (rp->Kr*rp->error);

	//summator update
	rp->sum += (rp->Kr * rp->Ti * rp->error) + (rp->Kip * rp->saturationDiff);
#endif

	// modified PID algorithm
#if REG_VERSION == 2
	//! not test
	// regulator output (action value)
	flOut = rp->Kr *((rp->Beta*rp->desired) - rp->measured)
		  + rp->sum
		  + rp->N*rp->Kr*( -rp->measured +( rp->der *(-1 + exp(( -0.02*rp->N)/rp->Td) )));
	//! TODO konstanta periody regulátoru

	// integration update
	rp->sum += (rp->Kr * rp->Ti * rp->error) + (rp->Kip * rp->saturationDiff / rp->outputScale);

	// derivation update
	rp->der = (rp->der * exp((-(1/SPEED_REG_FREQ)*rp->N)/rp->Td)) - rp->measured;
#endif

	//v tomto místì bude manuální/automatický režim

	rp->action = (short)flOut;

	// converter gain correction
	//rp->action = siOut = (short)( flOut ) * 538.0 / rp->batt_voltage;
	siOut = (short)( flOut * rp->outputScale);

	// output non-linearity saturation model
	if (siOut >  rp->limit)
		siOut =  rp->limit;
#if REG_UNIPOLAR_LIMIT == 1
	if (siOut < 0 )
		siOut = 0;
#else
	if (siOut < -rp->limit)
		siOut = -rp->limit;
#endif

	rp->action = siOut;

	// over integration protection feedback
	rp->saturationDiff = siOut - rp->action;

	return siOut;
}

void RegulatorSetPID(struct RegulatorParams * rp, float Kr, float Ti, float Td)
{
	// we only transfer values into structure
	rp->Kr = Kr;
	rp->Ti = Ti;
	rp->Td = Td;
}

void RegulatorSetParams(struct RegulatorParams * rp, float Beta, float Kip, float N)
{
	// we only transfer values into structure
	rp->Beta = Beta;
	rp->Kip = Kip;
	rp->N = N;
}

void RegulatorSetScaleLimit(struct RegulatorParams * rp, float outputScale, unsigned short limit)
{
	// we only transfer values into structure
	rp->limit = limit;
	rp->outputScale  = outputScale;
}
