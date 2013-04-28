#include "SimpleMatrix.h"

#ifndef FREERTOS

#include <stdio.h>
#include <stdlib.h>
#define pvPortMalloc(x) malloc(x)
#define vPortFree(x) free(x)

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
			printf("%5.2e ",mt->mat[j+(i*mt->n)]);
		}
		printf("\r\n");
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

void matTest( void)
{

	matrixType* a;
	matrixType* b;
	matrixType* c;

	a = matAlloc(2,3);
	b = matAlloc(3,2);
	c = matAlloc(2,2);

	matEye(a,2);
	matFill(b,3);

	matPrint(a);
	matPrint(b);
	matMul(c,a,b);
	matPrint(c);

	matPrint(a);
	CELL(a,0,0) = 10.0;
	CELL(a,1,2) = CELL(a,0,0);
	matPrint(a);

	matFree(a);
	matFree(b);
	matFree(c);

	/*
	matrixType* a;
	matrixType* b;
	matrixType* c;
	matrixType* d;
	matrixType* e;

	a = matAlloc(2,3);
	b = matAlloc(3,2);
	c = matAlloc(2,2);
	d = matAlloc(2,4);
	e = matAlloc(2,4);

	//matEye(a,2);
	//matFill(b,3);
	matFillTestPattern(a);
	matFillTestPattern(b);
	matFill(d,1.0);
	matPrint(a);
	matPrint(b);

	matMul(c,a,b);
	matPrint(c);

	matMul3(e,a,b,d);
	matPrint(e);
	*/
}

#else

#include "FreeRTOS.h"

#endif

matrixType * matAlloc(matrixSizeType row, matrixSizeType column)
{
	matrixType * temp;
	if ((row<1) || (column<1))
		return NULL;

	// first allocate memory for data structure
	temp = pvPortMalloc(sizeof(matrixType));
	if (temp == NULL)
		return temp;
	else
	{
		// when successful, allocate matrix items
		temp->mat = pvPortMalloc(row*column*sizeof(matrixValType));
		if (temp->mat == NULL)
			return NULL;
		else
		{
			// matrix dimensions settrs
			temp->m = row;
			temp->n = column;
			// return of new structure pointer
			return temp;
		}
	}
}

void matFree(matrixType *m)
{
	//
	vPortFree(m->mat);
	vPortFree(m);
}

inline matrixSizeType matAdd(matrixType *a, matrixType *b)
{
	matrixSizeType i, j;
#ifdef MATRIX_CHECK_DIMENSIONS
	if ((a->m!=b->m)||(a->n!=b->n))
		return -1;
#endif
	//for each row
	for( i = 0; i<(a->m); i++)
	{
		// for each cell(column) in row
		for( j = 0; j < (a->n) ; j++)
		{
			a->mat[j+(i*a->n)] += b->mat[j+(i*b->n)];
		}
	}
	return 0;
}

inline matrixSizeType matSubtract(matrixType *a, matrixType *b)
{
	matrixSizeType i, j;
#ifdef MATRIX_CHECK_DIMENSIONS
	if ((a->m!=b->m)||(a->n!=b->n))
		return -1;
#endif
	//for each row
	for( i = 0; i<(a->m); i++)
	{
		// for each cell(column) in row
		for( j = 0; j < (a->n) ; j++)
		{
			a->mat[j+(i*a->n)] -= b->mat[j+(i*b->n)];
		}
	}
	return 0;
}

inline matrixSizeType matMul(matrixType *res, matrixType *a, matrixType *b)
{
	matrixSizeType i, j, k;
#ifdef MATRIX_CHECK_DIMENSIONS
	if ((res->m!=a->m)||(res->n!=b->n)||(a->n!=b->m))
		return -1;
#endif
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
	return 0;
}

inline matrixSizeType matMul3(matrixType *res, matrixType *a, matrixType *b, matrixType *c)
{
	matrixSizeType i, j, k;
	matrixValType temp[MATRIX_TEMPORARY_MAX_SIZE];
#ifdef MATRIX_CHECK_DIMENSIONS
	if ((res->m!=a->m)||(res->n!=c->n)||(a->n!=b->m)||(b->n!=c->m)||(b->n>MATRIX_TEMPORARY_MAX_SIZE))
		return -1;
#endif
	//for each row
	for( i = 0; i<(res->m); i++)
	{
		// for each cell(column) in row mat b
		for( j = 0; j < (b->n) ; j++)
		{
			temp[j] = 0;
			for ( k = 0; k < (a->n); k++)
			{
				temp[j] += a->mat[k+(i*a->n)] * b->mat[j+(k*b->n)];
			}
		}
		// for each cell(column) in row mat c
		for( j = 0; j < (c->n) ; j++)
		{
			res->mat[j+(i*res->n)] = 0;
			for ( k = 0; k < (c->m); k++)
			{
				res->mat[j+(i*res->n)] += temp[k] * c->mat[j+(k*c->n)];
			}
		}
	}
	return 0;
}

inline matrixSizeType matTranspose(matrixType *res, matrixType *a)
{
	matrixSizeType i, j;
#ifdef MATRIX_CHECK_DIMENSIONS
	if ((res->m!=a->n)||(res->n!=a->m))
		return -1;
#endif
	//for each row
	for( i = 0; i<(res->m); i++)
	{
		// for each cell(column) in row
		for( j = 0; j < (res->n) ; j++)
		{
			res->mat[j+(i*res->n)] = a->mat[i+(j*a->n)];
		}
	}
	return 0;
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

inline matrixSizeType matDivConst(matrixType *a, matrixValType value)
{
	matrixSizeType i, j;

#ifdef MATRIX_CHECK_DIMENSIONS
	if (value == 0)
		return -1;
#endif

	//for each row
	for( i = 0; i<(a->m); i++)
	{
		// for each cell(column) in row
		for( j = 0; j < (a->n) ; j++)
		{
			a->mat[j+(i*a->n)] /= value;
		}
	}
	return 0;
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

inline matrixSizeType matInv(matrixType* res, matrixType* a)
{
#ifdef MATRIX_CHECK_DIMENSIONS
	if ((a->m != a->n)||(res->m!=res->n))
		return -1;
#endif
	// initially, resulting matrix is diagonal
	//mathEye(res,1);??

	// gauss jordan elimination method for math [ a | res ]



	return 0;
}

