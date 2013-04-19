
#ifndef __RLSE_H__
#define __RLSE_H__

#include "SimpleMatrix.h"

#define RLSE_SYSTEM_PARAMETERS (4)
#define EXP_FORGETING_COEF (0.99)

typedef struct rlseStruct
{
	// ! past values vector
	matrixValType past_values[RLSE_SYSTEM_PARAMETERS-1];

	// ! RLS method matrices
	matrixType* P;
	matrixType* th;
	matrixType* K;
	matrixType* phi;
}
rlseType;

enum PAST_VALS_ID
{
	Y1 = 0, Y2, U1
};

/*
matrixType* P;
matrixType* th;
matrixType* K;
matrixType* phi;*/

inline void matPrint( matrixType *mt);
void matTest( void);
void rmnc_init( rlseType* rlses);
void rmnc_reinit( rlseType* rlses);
void rmnc_update( rlseType* rlses, matrixValType y, matrixValType u, matrixSizeType condition);

#endif//__RLSE_H__
