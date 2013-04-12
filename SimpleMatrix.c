#include "SimpleMatrix.h"

#if 1
#include "FreeRTOS.h"

#else
#include <stdio.h>
#include <stdlib.h>
#error WIN
#define pvPortMalloc(x) malloc(x)

inline void matPrint( matrixType *mt)
{
	matrixSizeType i, j;
	printf("\r\n");
	//for each row
	for( i = 0; i<(mt->m); i++)
	{

		// for each cell(column) in row
		for( j = 0; j < (mt->n) ; j++)
		{
			printf("%7.4f ",mt->mat[j+(i*mt->n)]);
		}
		printf("\r\n");
	}
}

#endif

matrixType * matAlloc(matrixSizeType m, matrixSizeType n)
{
	matrixType * temp;
	temp = pvPortMalloc(sizeof(matrixType));
	if (temp == NULL)
		return temp;
	else
	{
		temp->mat = pvPortMalloc(m*n*sizeof(matrixValType));
		if (temp->mat == NULL)
			return NULL;
		else
		{
			temp->m = m;
			temp->n = n;
			return temp;
		}
	}
}



inline void matAdd(matrixType *a, matrixType *b)
{
	matrixSizeType i, j;

	//for each row
	for( i = 0; i<(a->m); i++)
	{
		// for each cell(column) in row
		for( j = 0; j < (a->n) ; j++)
		{
			a->mat[j+(i*a->n)] += b->mat[j+(i*b->n)];
		}
	}
}

inline void matSubstract(matrixType *a, matrixType *b)
{
	matrixSizeType i, j;
	//for each row
	for( i = 0; i<(a->m); i++)
	{
		// for each cell(column) in row
		for( j = 0; j < (a->n) ; j++)
		{
			a->mat[j+(i*a->n)] -= b->mat[j+(i*b->n)];
		}
	}
}

inline void matMul(matrixType *res, matrixType *a, matrixType *b)
{
	matrixSizeType i, j, k;
	//for each row
	for( i = 0; i<(res->m); i++)
	{
		// for each cell(column) in row
		for( j = 0; j < (res->n) ; j++)
		{
			res->mat[j+(i*res->n)] = 0;
			for ( k = 0; k < (a->n); k++)
			{
				res->mat[j+(i*res->n)] += a->mat[k+(i*a->n)] * b->mat[j+(k*b->n)];
			}
		}
	}
}

inline void matTranspose(matrixType *res, matrixType *a)
{
	matrixSizeType i, j;
	//for each row
	for( i = 0; i<(res->m); i++)
	{
		// for each cell(column) in row
		for( j = 0; j < (res->n) ; j++)
		{
			res->mat[j+(i*res->n)] = a->mat[i+(j*a->n)];
		}
	}
}

inline void matScale(matrixType *a, matrixValType scale)
{
	matrixSizeType i, j;
	//for each row
	for( i = 0; i<(a->m); i++)
	{
		// for each cell(column) in row
		for( j = 0; j < (a->n) ; j++)
		{
				a->mat[j+(i*a->n)] *= scale;
		}
	}
}

inline void matDivConst(matrixType *a, matrixValType value)
{
	matrixSizeType i, j;
	//for each row
	for( i = 0; i<(a->m); i++)
	{
		// for each cell(column) in row
		for( j = 0; j < (a->n) ; j++)
		{
				a->mat[j+(i*a->n)] /= value;
		}
	}
}

inline void matAddConst(matrixType *a, matrixValType value)
{
	matrixSizeType i, j;
	//for each row
	for( i = 0; i<(a->m); i++)
	{
		// for each cell(column) in row
		for( j = 0; j < (a->n) ; j++)
		{
				a->mat[j+(i*a->n)] += value;
		}
	}
}

inline void matFill(matrixType *res, matrixValType value)
{
	matrixSizeType i;
	//for each row
	for( i = 0; i<(res->m*res->n); i++)
	{
			res->mat[i] = value;
	}
}

inline void matEye(matrixType* res, matrixValType value)
{
	matrixSizeType i, j;
	//for each row
	for( i = 0; i<(res->m); i++)
	{
		// for each cell(column) in row
		for( j = 0; j < (res->n) ; j++)
		{
			if (i==j)
				res->mat[j+(i*res->n)] = value;
			else
				res->mat[j+(i*res->n)] = 0;
		}
	}
}
inline void matFillTestPattern(matrixType *res)
{
	matrixSizeType i;
	//for each row
	for( i = 0; i<(res->m*res->n); i++)
	{
			res->mat[i] = i+1;
	}
}



matrixValType ya1 = 0.0;
matrixValType y2 = 0.0;
matrixValType u1 = 0.0;


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
		matSubstract(rlses->P,temp_nn_2);

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



void matTest( void)
{
	/*matrixType* a;
	matrixType* b;
	matrixType* c;

	a = matAlloc(3,3);
	b = matAlloc(3,3);
	c = matAlloc(3,3);

	matEye(a,2);
	matFill(b,3);

	matAdd(c,a,b);*/

	/*matrixType* a;
	matrixType* aT;
	matrixType* b;
	matrixType* c;

	a = matAlloc(3,2);
	aT = matAlloc(2,3);
	b = matAlloc(3,2);
	c = matAlloc(2,2);

	//matEye(a,2);
	//matFill(b,3);
	matFillTestPattern(a);
	matFillTestPattern(b);

	matTranspose(aT,a);

	matMul(c,aT,b);*/
}

