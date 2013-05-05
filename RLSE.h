
#ifndef __RLSE_H__
#define __RLSE_H__

#include <math.h>

#include "SimpleMatrix.h"

#define RLSE_MINIMAL_UPDATES (25)
#define RLSE_EXP_FORGETING_COEF (0.98)
#define RLSE_P_MATRIX_INIT	(1.0e1)
#define RLSE_IMV_TYPE (1)


#define RLSE_SYSTEM_PARAMETERS (4)

typedef struct regParamsStruct
{
	float Kkrit;
	float Tkrit;

	float Kr;
	float Ti;
	float Td;
}
regParamType;


typedef struct rlseStruct
{
	//! past values vector
	matrixValType past_values[RLSE_SYSTEM_PARAMETERS+1];

	//! minimal identification step counter
	matrixSizeType condition;

	// ! RLS method matrices
	matrixType* P;
	matrixType* th;
	matrixType* K;
	matrixType* phi;

#if RLSE_IMV_TYPE == 1
	matrixType* DZ;
#endif

	matrixType delta;
}
rlseType;


enum PAST_VALS_ID
{
	Y1 = 0, Y2, Y3, Y4, U1
};

inline void matPrint( matrixType *mt);
void matTest( void);
void rlse_init( rlseType* rlses);
void rlse_reinit( rlseType* rlses);
void rlse_update( rlseType* rlses, matrixValType y, matrixValType u, matrixSizeType condition);

void compute_params( matrixType *th, regParamType *reg);

#endif//__RLSE_H__
