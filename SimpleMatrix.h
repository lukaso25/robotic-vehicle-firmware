
#ifndef __SIMPLE_MATRIX_H__
#define __SIMPLE_MATRIX_H__

#include <stdlib.h>

typedef float  matrixValType;
typedef long matrixSizeType;

typedef struct matrixTypeStruct
{
	//pointer to linear array holding 2 dimensional matrix
	matrixValType * mat;
	// column count
	matrixSizeType m;
	// row count
	matrixSizeType n;
}
matrixType;

matrixType * matAlloc(matrixSizeType m, matrixSizeType n);

inline void matAdd(matrixType *a, matrixType *b);

inline void matSubstract(matrixType *a, matrixType *b);

inline void matMul(matrixType *res, matrixType *a, matrixType *b);

inline void matTranspose(matrixType *res, matrixType *a);

inline void matScale(matrixType *a, matrixValType scale);

inline void matDivConst(matrixType *a, matrixValType value);

inline void matAddConst(matrixType *a, matrixValType value);

inline void matFill(matrixType *res, matrixValType value);

inline void matEye(matrixType* res, matrixValType value);

#endif//__SIMPLE_MATRIX_H__



