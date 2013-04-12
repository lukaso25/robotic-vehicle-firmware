
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
void rmnc_update( rlseType* rlses, matrixValType y, matrixValType u, matrixSizeType condition);
