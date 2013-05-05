#ifndef __ACCELEROMETER_H__
#define __ACCELEROMETER_H__

#include "FreeRTOS.h"
#include "CommonDefs.h"

/*! \file Accelerometer.h
 *  \author Lukas Otava
 *  \date 2013
 *  \defgroup Accelerometer Accelerometer module
 *
 *
 *
 * */

short int accData[2];


signed portBASE_TYPE  AccelerometerInit(  unsigned portBASE_TYPE priority);

signed portBASE_TYPE AccelerometerRequestData( void);

void Accelerometer_task( void * pvParameters );

#endif//__ACCELEROMETER_H__
