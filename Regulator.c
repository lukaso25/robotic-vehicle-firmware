#include "Regulator.h"


short SpeedReg(struct RegulatorParams * rp, short measurement)
{
	float flOut;
	short siOut;

	rp->actual = measurement;
	// error value
	rp->error = rp->desired - measurement;

	//regulator output
	flOut = rp->sum + (rp->K*rp->error);

	//summator update
	rp->sum += (rp->K * rp->Ti * rp->error) + (rp->Kip * rp->saturationDiff);

	// integration value limitation
	/*if (rp->sum  > SUM_LIMIT)
		rp->sum  = SUM_LIMIT;
	if (rp->sum  < -SUM_LIMIT)
		rp->sum  = -SUM_LIMIT;*/

	// converter gain correction
	rp->action = siOut = (short)( flOut * 538.0 / rp->batt_voltage);

	// output non-linearity saturation model
	if (siOut >  rp->limit)
		siOut =  rp->limit;
	if (siOut < -rp->limit)
		siOut = -rp->limit;

	// over integration protection feedback
	rp->saturationDiff = siOut - rp->action;

	return siOut;
}


