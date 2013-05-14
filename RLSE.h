
#ifndef __RLSE_H__
#define __RLSE_H__

#include <math.h>

#include "SimpleMatrix.h"

/*! \file RLSE.h
 * \author Lukas Otava
 * \date 2013
 * \defgroup RLSE RLSE module
 *
 * Recursive least square identificaton module
 */

#define RLSE_MINIMAL_UPDATES (19)
#define RLSE_EXP_FORGETING_COEF (0.98)
#define RLSE_P_MATRIX_INIT	(1.0e8)
#define RLSE_IMV_TYPE (0)


#define RLSE_SYSTEM_PARAMETERS (4)

typedef struct rlseStruct
{
	//! past values vector
	matrixValType past_values[RLSE_SYSTEM_PARAMETERS+1];

	//! minimal identification step counter
	matrixSizeType condition;

	// ! RLS matrix P
	matrixType* P;
	// ! RLS matrix theta
	matrixType* th;
	// ! RLS matrix K
	matrixType* K;
	// ! RLS matrix phi
	matrixType* phi;

#if RLSE_IMV_TYPE == 1
	matrixType* DZ;
#endif

	// ! RLS
	matrixType delta;
}
rlseType;

typedef struct regParamsStruct
{
	float Kkrit;
	float Tkrit;

	float Kr;
	float Ti;
	float Td;
}
regParamType;

enum PAST_VALS_ID
{
	Y1 = 0, Y2, Y3, Y4, U1
};



void rlse_init( rlseType* rlses);

void rlse_reinit( rlseType* rlses);

void rlse_update( rlseType* rlses, matrixValType y, matrixValType u, matrixSizeType condition);

void compute_params( matrixType *th, regParamType *reg);

#endif//__RLSE_H__
