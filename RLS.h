
#ifndef __RLSE_H__
#define __RLSE_H__

#include <math.h>

#include "SimpleMatrix.h"

/*! \file RLS.h
 * \author Lukas Otava
 * \date 2013
 * \defgroup RLS RLS module
 *
 * This is the recursive least square method identification module.
 *
 * This are parameters of identification:
 *
 * \f$
 *  y_{(k)} = \varphi_{(k)}^T \theta_{(k)} + e_{s(k)}
 * \f$

	where

	\f$ \varphi_{(k)}^T =
			\left(\begin{array}{ccccccc}
					 -y_{(k-1)} & -y_{(k-2)} & u_{(k-1)} & u_{(k-2)})
			\end{array}\right)
	\f$

	observation vector


	\f$	\theta_{(k)}^T =
			\left(\begin{array}{ccccccc}
					 a_1 & a_2 & b_1 & b_2
			\end{array}\right)
			\f$
 *
 * * Example usage:
   \code{c}
  	rlsType ident;

	RLS_init(&ident);

	while (1)
	{
		int condition = 0;
		if (changes in process)
			condition = 1;
		// with new sample
		waitNewData();
		// compute parameters update
		RLS_update(&reg, measured_value, condition);
	}

	// params recall in matrix
	ident.th
	\endcode
 *
 */

//! \brief Minimal identification update count (after the latest condition==1)
//! \ingroup  RLS
#define RLS_MINIMAL_UPDATES (10)

//! \brief Exponential forgetting coefficient
//! \ingroup  RLS
#define RLS_EXP_FORGETING_COEF (0.99)

//! \brief Default values of the Covariance matrix main diagonal
//! \ingroup  RLS
#define RLS_P_MATRIX_INIT	(1.0e8)

//! \brief This macro choose modified recursive least squares method with specific setting
//! \ingroup  RLS
#define RLS_IMV_TYPE (0)

//! \brief Numer of the identified parameters
//! \ingroup  RLS
#define RLS_SYSTEM_PARAMETERS (4)

//! \ingroup  RLS
//! Structure holding data of RLS identification algorithm
typedef struct rlsStruct
{
	//! past values vector
	matrixValType past_values[RLS_SYSTEM_PARAMETERS+1];

	//! minimal identification step counter
	matrixSizeType condition;

	//! RLS matrix P
	matrixType* P;
	//! RLS matrix theta
	matrixType* th;
	//! RLS matrix K
	matrixType* K;
	//! RLS matrix phi
	matrixType* phi;

#if RLSE_IMV_TYPE == 1
	matrixType* DZ;
#endif

	//! RLS parameter delta
	matrixType delta;
}
rlsType;

//! \brief This is past parameters enumeration
//! \ingroup RLS
enum PAST_VALS_ID
{
	Y1 = 0, Y2, Y3, Y4, U1
};

/*! \brief Module initialization function
 * \param rlses Pointer to identification structure
* \ingroup  RLS
* */
void RLS_init( rlsType* rlses);

/*! \brief Module parameters re-initialization
 * \param rlses Pointer to identification structure
* \ingroup  RLS
* */
void RLS_reinit( rlsType* rlses);

/*! \brief Module update function
 *
 * This function compute parameters of system according this equations:
 *
 * \f$

			K_{(k+1)} = P_{(k)} \varphi_{(k+1)} \left[ \lambda_e + \varphi_{(k+1)}^T P_{(k)} \varphi_{(k+1)} \right]^{-1}
\f$

 \f$
			\theta_{(k+1)} = \theta_{(k)} + K_{(k+1)}(y_{(k+1)} - \varphi_{(k+1)}^T \theta_{(k)} )
\f$

 \f$
			P_{(k+1)} =  \left( P_{(k)} -  K_{(k+1)} \varphi_{(k+1)}^T  P_{(k)} \right) \frac{1}{\lambda_e}
 * \f$
 *
 * \param rlses Pointer to identification structure
 * \param y Measured value on input of the identified system
 * \param u Value on input of the identified system
 * \param condition Condifion for system identification
*  \ingroup  RLS
* */
void RLS_update( rlsType* rlses, matrixValType y, matrixValType u, matrixSizeType condition);


#endif//__RLSE_H__
