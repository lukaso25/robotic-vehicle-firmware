/*! \mainpage Robotic vehicle API Documentation main Page
 *
 *
 * This is the introduction of project.
 *
 *
 *
 *
 * Project structure
 * -----------------
 *
 *
 * Module structure image
 *
 * Modules that is contained in firmware is in Module Tab.
 *
 * Simple project structure
 * ------------------------
 * User application should be created as FreeRTOS task. This task can use other loaded modules.
 *
 *  * This is example of main function:
 *  \code{c}
int main(void)
{
	//! system clock initialization
	vSystemInit();

	//! HeartBeat
	if (StatusLEDInit(tskIDLE_PRIORITY+1) != pdPASS)
	{
		//error
	}

	//! SlipSerial communication
	if (SlipSerialInit(tskIDLE_PRIORITY +2, 57600, 200) != pdPASS)
	{
		//error
	}

	//! MotorControl module initialization
	if (MotorControlInit(tskIDLE_PRIORITY+4) != pdPASS)
	{
		// error occurred during MotorControl module initialization
	}

	//! accelerometer task
	if (AccelerometerInit(tskIDLE_PRIORITY +3) != pdPASS)
	{
		//error
	}

	//! Control task
	if ( ControlTaskInit(tskIDLE_PRIORITY +2) != pdPASS)
	{
		//error
	}

	//! CANTest task
	if (xTaskCreate(CANtest_task, (signed portCHAR *) "CAN", 256, NULL, tskIDLE_PRIORITY +1, NULL) != pdPASS)
	{
		//error
	}

	//! USER DEFINED TASK

#if configUSE_TRACE_FACILITY==1
	//! FreeRTOs kernel object trace record start
	if (uiTraceStart() != 1)
		while(1){};
#endif

	//! FreeRTOS scheduler start
	vTaskStartScheduler();

	//! never entering code
	while(1){};
	return 0;
}
	\endcode
 *
 * How to add new FreeRTOS task
 * ----------------------------
 *
 * How to use trace view?
 * -----------------------
 * 1. FreeRTOS heap and task stacks should be configured with spare size.
 * 2. configUSE_TRACE_FACILITY in FreeRTOSConfig.h must be set to 1
 * 3. While debugging you can download SRAM image (called memory dump) (In CoIDE - Memory card)
 * 4. This dump file can be opened in Percepio FreeRTOS+Trace
 *
 * How configure heap size?
 * ------------------------
 * During debugging you can display value of xFreeBytesRemaining global variable, that express free
 *
 *
 *
 * Lukáš Otava 2013

 */
/*
 *  * TODO: upravit kód regulátoru
 *
 * TODO: upravit RLS
 *
 * TODO: vytvoøit kód pro jízdu po zadané trajektorii
 *
 * TODO: k èemu bude identifikace? když možná špatnì identifikuje statické zesílení?
 *
 * TODO:
 *
 * */

/* \file
 * \author Lukas Otava
 * \date 2013
 * \defgroup  */


/* \brief
 *
 * Example usage:
 *  \code{c}

	\endcode
 * \param
 * \ingroup
 * \return
 * \warning */
