#include "Regulator.h"

#include <math.h>

short RegulatorAction(struct RegulatorParams * rp, short measurement)
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
	rp->sum += (rp->Kr * rp->Ti * rp->error) + (rp->Kip * rp->saturationDiff);

	// derivation update
	rp->der = (rp->der * exp((-0.02*rp->N)/rp->Td)) - rp->measured;
#endif

	// converter gain correction
	rp->action = siOut = (short)( flOut ) * 538.0 / rp->batt_voltage;

	// output non-linearity saturation model
	if (siOut >  rp->limit)
		siOut =  rp->limit;
	if (siOut < -rp->limit)
		siOut = -rp->limit;

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

void RegulatorSetParam(struct RegulatorParams * rp, float Beta, float Kip, float N)
{
	// we only transfer values into structure
	rp->Beta = Beta;
	rp->Kip = Kip;
	rp->N = N;
}

void RegulatorSetLimit(struct RegulatorParams * rp, short limit)
{
	// we only transfer values into structure
	rp->limit = limit;
}


