#include "SimpleMatrix.h"

#ifdef FREERTOS
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

