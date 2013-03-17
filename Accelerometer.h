#ifndef __ACCELEROMETER_H__
#define __ACCELEROMETER_H__

#include "FreeRTOS.h"

signed portBASE_TYPE  AccelerometrInit(  unsigned portBASE_TYPE priority);

void Accelerometer_task( void * pvParameters );

#endif//__ACCELEROMETER_H__
