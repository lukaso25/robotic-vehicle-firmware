
#ifndef __RLSE_H__
#define __RLSE_H__

#include <math.h>

#include "SimpleMatrix.h"

#define RLSE_SYSTEM_PARAMETERS (4)
#define RLSE_MINIMAL_UPDATES (10)
#define EXP_FORGETING_COEF (0.98)

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
	// ! past values vector
	matrixValType past_values[RLSE_SYSTEM_PARAMETERS];

	matrixSizeType condition;

	// ! RLS method matrices
	matrixType* P;
	matrixType* th;
	matrixType* K;
	matrixType* phi;
	matrixType* DZ;

	matrixType delta;
}
rlseType;

enum PAST_VALS_ID
{
	Y1 = 0, Y2, Y3, U1
};

/*
matrixType* P;
matrixType* th;
matrixType* K;
matrixType* phi;*/

inline void matPrint( matrixType *mt);
void matTest( void);
void rlse_init( rlseType* rlses);
void rlse_reinit( rlseType* rlses);
void rlse_update( rlseType* rlses, matrixValType y, matrixValType u, matrixSizeType condition);

void compute_params( matrixType *th, regParamType *reg);

#endif//__RLSE_H__
