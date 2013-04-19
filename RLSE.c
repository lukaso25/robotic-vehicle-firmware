
#include "RLSE.h"

matrixType* phiT = NULL;
matrixType* temp_1n = NULL;
matrixType* temp11 = NULL;
matrixType* temp_nn = NULL;
matrixType* temp_nn_2 = NULL;

// wars - all system must be same order :-( in consequence to shared temporary matrices
void rmnc_init( rlseType* rlses)
{
	//iterator variable declaration
	matrixSizeType i;

	// individual rls matrix allocation
	rlses->P = matAlloc(RLSE_SYSTEM_PARAMETERS,RLSE_SYSTEM_PARAMETERS);
	rlses->th  = matAlloc(RLSE_SYSTEM_PARAMETERS,1);
	rlses->K   = matAlloc(RLSE_SYSTEM_PARAMETERS,1);
	rlses->phi = matAlloc(RLSE_SYSTEM_PARAMETERS,1);

	// shared temporary matrix allocation
	if (phiT == NULL)
	{
		phiT   = matAlloc(1,RLSE_SYSTEM_PARAMETERS);
		temp_1n = matAlloc(1,RLSE_SYSTEM_PARAMETERS);
		temp11    = matAlloc(1,1);
		temp_nn  = matAlloc(RLSE_SYSTEM_PARAMETERS,RLSE_SYSTEM_PARAMETERS);
		temp_nn_2  = matAlloc(RLSE_SYSTEM_PARAMETERS,RLSE_SYSTEM_PARAMETERS);
	}

	// matrix initialization
	matEye(rlses->th, 1.0);
	matEye(rlses->P,  1.0e9);

	// past value vector init
	for( i = 0; i < (RLSE_SYSTEM_PARAMETERS-1); i++)
	{
		rlses->past_values[i] = 0;
	}
}

void rmnc_update( rlseType* rlses, matrixValType y, matrixValType u, matrixSizeType condition)
{
	matrixValType eps;

	//update vstupù
	rlses->phi->mat[0] = -rlses->past_values[Y1];//ya1;
	rlses->phi->mat[1] = -rlses->past_values[Y2];
	rlses->phi->mat[2] =  u;
	rlses->phi->mat[3] =  rlses->past_values[U1];

	if (condition)
	{
		matTranspose(phiT,rlses->phi);
		//matPrint(phi);
		//matPrint(phiT);

		//epsilon
		matMul(temp11,phiT,rlses->th);
		eps = y - temp11->mat[0];
		//matPrint(theta);
		//matPrint(one);
		//printf(" %5.2f ",eps);

		//K
		matMul(temp_1n,phiT,rlses->P);
		matMul(temp11,temp_1n,rlses->phi);
		temp11->mat[0] += EXP_FORGETING_COEF; //(phi'*P*phi)
		matMul(rlses->K,rlses->P,rlses->phi);
		matDivConst(rlses->K,temp11->mat[0]);

		//P
		matMul(temp_nn,rlses->K,phiT);
		matMul(temp_nn_2,temp_nn,rlses->P);
		matSubtract(rlses->P,temp_nn_2);

		matDivConst(rlses->P,EXP_FORGETING_COEF);

		//theta - ok
		matScale(rlses->K,eps);
		matAdd(rlses->th, rlses->K);
	}

	// update minulých vstupù
	rlses->past_values[Y2] = rlses->past_values[Y1];
	rlses->past_values[Y1] = y;
	rlses->past_values[U1] = u;
}
