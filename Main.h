/*! \mainpage Robotic vehicle API Documentation main Page
 *
 *
 *
 * Project structure
 * -----------------
 *
 *On this picture there are relations between program modules that comprise the general firmware of robotic vehicle. Blocks are clickable.
 *\dot
  digraph example {
      node [shape=record, fontname=Sans, fontsize=11];
      fr [ label="FreeRTOS"];
      dl [ label="DriverLib"];
      cd [ label="CommonDefs" URL="\ref CommonDefs"];
      sm [ label="SimpleMatrix" URL="\ref SimpleMatrix"];
      sl [ label="StatusLED" URL="\ref StatusLED"];
      rlse [ label="RLS" URL="\ref RLS"];
      reg [ label="Regulator" URL="\ref Regulator"];
      mot [ label="MotorControl" URL="\ref MotorControl"];
      acc [ label="Accelerometer" URL="\ref Accelerometer"];
      can [ label="CANtest" URL="\ref CANtest"];
      sl -> fr [ arrowhead="open", style="dashed" ];
      sl -> dl [ arrowhead="open", style="dashed" ];
      sm -> fr [ arrowhead="open", style="dashed" ];
      rlse -> sm [ arrowhead="open", style="dashed" ];
      mot -> rlse [ arrowhead="open", style="dashed" ];
      mot -> reg [ arrowhead="open", style="dashed" ];
      mot -> sl [ arrowhead="open", style="dashed" ];
      acc -> sl [ arrowhead="open", style="dashed" ];
      can -> sl [ arrowhead="open", style="dashed" ];
  }
  \enddot

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
 * How to change microcontroller clock frequency
 * ---------------------------------------------
 * There are two steps:
 * 1. You must change Divider value of SysCtlClockSet() in vSystemInit() in Main.c
 * 2. You must change clock frequency in FreeRTOSConfig.h
 *
 * How to add new FreeRTOS task
 * ----------------------------
 *\code{c}
 void UserTask_task( void * param)
  {
  	while(1)
  	{
  	//empty task
  	vTaskDelay(10);
  	}
  }
 * .....
 * xTaskCreate(UserTask_task, (signed portCHAR *) "USER", 512, NULL, tskIDLE_PRIORITY , NULL)
 * \endcode
 *
 * How to change MCU pin mapping
 * -----------------------------
 * Each used microcontroller pin is defined in CommonDefs.h.
 *
 * How to use Trace view?
 * -----------------------
 * 1. FreeRTOS heap and task stacks should be configured with spare size.
 * 2. configUSE_TRACE_FACILITY in FreeRTOSConfig.h must be set to 1
 * 3. While debugging you can download SRAM image (called memory dump) (In CoIDE - Memory card)
 * 4. This dump file can be opened in Percepio FreeRTOS+Trace
 *
 * How configure heap size?
 * ------------------------
 * During debugging you can display value of xFreeBytesRemaining global variable, that express free heap memory.
 *
 * How to check stack usage?
 * ------------------------
 * You should use FreeRTOS function vTaskList() that return task list with the actual task stack usage.
 *
 * License
 * -------
 * License is specified in Licence.txt file.
 *
 * Brno, Lukáš Otava 2013

 */




