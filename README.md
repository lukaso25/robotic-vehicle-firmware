/*! \mainpage Robotic vehicle API Documentation main Page
 *  \author Lukas Otava
 *
 * This is the introduction of project.
 *
 * This base firmware contain many modules.
 *
 * Simple project structure
 * ------------------------
 * User application should be created as FreeRTOS task. This task can use other loaded modules.
 *
 *  * This is example of main function:
 *  \code{c}
int main(void)
{
	//! inicializace z�kladn�ch periferi� syst�mu
	vSystemInit();

	//! HeartBeat �loha indika�n� diody
	if (StatusLEDInit(tskIDLE_PRIORITY+1) != pdPASS)
	{
		//error
	}

	//! SlipSerial komunikace po seriov� lince
	if (SlipSerialInit(tskIDLE_PRIORITY +2, 57600, 200) != pdPASS)
	{
		//error
	}

	//! MotorControl module initialization
	if (MotorControlInit(tskIDLE_PRIORITY+4) != pdPASS)
	{
		// error occurred during MotorControl module initialization
	}

	//! �loha akcelerometru
	if (AccelerometerInit(tskIDLE_PRIORITY +3) != pdPASS)
	{
		//error
	}

	//! �loha ��zen�  - tady budou kraviny kolem, logov�n� atd
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
 * TODO: upravit k�d regul�toru
 *
 * TODO: upravit RLS
 *
 * TODO: vytvo�it k�d pro j�zdu po zadan� trajektorii
 *
 * TODO: k �emu bude identifikace? kdy� mo�n� �patn� identifikuje statick� zes�len�?
 *
 * TODO:
 */


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



robotic-vehicle-firmware
========================


Based on 
- FreeRTOS 7.3.0
- TraceRecorder v2.2.3
- LM3Slib revision 7243
- CMSIS V1.30


Building environment
--------------------

- CoIDE 1.6.0
- gcc-arm-none-eabi-4_6-2012q2-20120614


