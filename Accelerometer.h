#ifndef __ACCELEROMETER_H__
#define __ACCELEROMETER_H__

#include "FreeRTOS.h"
#include "CommonDefs.h"

/*! \file Accelerometer.h
 *  \author Lukas Otava
 *  \date 2013
 *  \defgroup Accelerometer Accelerometer module
 *	This module is handling accelerometer connected through \f$I^2C\f$ interface.
 * */

//! \brief Accelerometer measured raw data array
//! \ingroup Accelerometer
short int accData[2];

//! \brief Accelerometer module initialization function
//! This function initiates MCU HW, Accelerometer setting and FreeRTOS objects.
//! \param priority FreeRTOS priority at which the task should run.
//! \return pdPass value is returned if module task was created correctly.
//! \warning This Module require FreeRTOS environment.
//! \ingroup Accelerometer
signed long  AccelerometerInit(  unsigned long priority);

//! \brief This function start new accelerometer measurement
//! \return pdTrue is returned if correct
//! \ingroup Accelerometer
signed long AccelerometerRequestData( void);

//! \brief This function returns value of actual Y acceleration in \f$m.s^2\f$
//! \return Actual X acceleration in \f$m.s^2\f$
//! \ingroup Accelerometer
float AccelerometerGetX( void);

//! \brief This function returns value of actual Y acceleration in \f$m.s^2\f$
//! \return Actual Y acceleration in \f$m.s^2\f$
//! \ingroup Accelerometer
float AccelerometerGetY( void);

//! \brief Accelerometer FreeRTOS task
//! \ingroup Accelerometer
void Accelerometer_task( void * pvParameters );

#endif//__ACCELEROMETER_H__
