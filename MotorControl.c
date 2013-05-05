#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_uart.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pwm.h"
#include "driverlib/qei.h"
#include "driverlib/adc.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "trcUser.h"

#include <math.h>

#include "MotorControl.h"
#include "Regulator.h"
#include "StatusLED.h"

xQueueHandle xSpeedActQ;
xQueueHandle xMotorPWMQ1;
xQueueHandle xMotorPWMQ2;

xSemaphoreHandle xWaitData;

void PWMFault_IRQHandler( void)
{
	//error
}

void InitADC( void)
{
	//  ADC
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

	ADCHardwareOversampleConfigure(ADC0_BASE, 64);

	ADCSequenceDisable(ADC0_BASE, 0);
	ADCSequenceConfigure(ADC0_BASE,0, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(ADC0_BASE,0,0,(ADC_CTL_CH0));
	ADCSequenceStepConfigure(ADC0_BASE,0,1,(ADC_CTL_CH1));
	ADCSequenceStepConfigure(ADC0_BASE,0,2,(ADC_CTL_CH2|ADC_CTL_END|ADC_CTL_IE));
	ADCSequenceEnable(ADC0_BASE,0);

	ADCProcessorTrigger(ADC0_BASE, 0);
}

long MeasureADC( void)
{
	unsigned long adcval[3];
	long count;

	while(!ADCIntStatus(ADC0_BASE, 0, false))
	{};

	count = ADCSequenceDataGet(ADC0_BASE,0,&adcval[0]);
	ADCProcessorTrigger(ADC0_BASE, 0);
	myDrive.mot1.current_total += myDrive.mot1.current_act = (short) adcval[0];
	myDrive.mot2.current_total += myDrive.mot2.current_act = (short) adcval[1];
	myDrive.batt_voltage = (short) adcval[2];

	return count;
}

unsigned long pwm_period = 0;
unsigned long qei_period = 0;

signed long MotorControlInit( unsigned long priority)
{
	// variables

	pwm_period =  SysCtlClockGet()/MOTOR_PWM_FREQ;
	qei_period = (SysCtlClockGet()/SPEED_REG_FREQ)-1;

	//Bridges GPIO init
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA|SYSCTL_PERIPH_GPIOB|SYSCTL_PERIPH_GPIOD|SYSCTL_PERIPH_GPIOE|SYSCTL_PERIPH_GPIOF|SYSCTL_PERIPH_GPIOG);

	//*********** PWM
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM);

	// Configure pins as PWM outs
	GPIOPinTypePWM(BRIDGE0_IN1_PORT,BRIDGE0_IN1);
	GPIOPinTypePWM(BRIDGE0_IN2_PORT,BRIDGE0_IN2);

	GPIOPinTypePWM(BRIDGE1_IN1_PORT,BRIDGE1_IN1);
	GPIOPinTypePWM(BRIDGE1_IN2_PORT,BRIDGE1_IN2);

#ifdef BIPOLAR_PWM_CONTROL

#else
	//konfigurace generátorù - možnosti úprav synchronizace (PWM_GEN_MODE_SYNC | PWM_GEN_MODE_GEN_SYNC_GLOBAL | PWM_GEN_MODE_GEN_SYNC_LOCAL)
	PWMGenConfigure(PWM_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC | PWM_GEN_MODE_DBG_RUN);
	PWMGenConfigure(PWM_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC | PWM_GEN_MODE_DBG_RUN);

	PWMGenPeriodSet(PWM_BASE, PWM_GEN_0, pwm_period);
	PWMGenPeriodSet(PWM_BASE, PWM_GEN_1, pwm_period);

	// widths init
	PWMPulseWidthSet(PWM_BASE, PWM_OUT_0, 1);
	PWMPulseWidthSet(PWM_BASE, PWM_OUT_1, 1);

	PWMPulseWidthSet(PWM_BASE, PWM_OUT_2, 1);
	PWMPulseWidthSet(PWM_BASE, PWM_OUT_3, 1);

	// this enables generators
	PWMGenEnable(PWM_BASE, PWM_GEN_0);
	PWMGenEnable(PWM_BASE, PWM_GEN_1);
#endif

	// output matrix
	//PWMOutputState(PWM_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT | PWM_OUT_2_BIT | PWM_OUT_3_BIT, false);// dopøedu
	//PWMOutputState(PWM_BASE, PWM_OUT_1_BIT | PWM_OUT_2_BIT, true);// dopøedu
	//PWMOutputState(PWM_BASE, PWM_OUT_0_BIT | PWM_OUT_3_BIT, true);// dopøedu
	//PWMOutputState(PWM_BASE, PWM_OUT_1_BIT | PWM_OUT_2_BIT, false);

	// inverted output select
	//PWMOutputInvert(PWM_BASE, (PWM_OUT_0_BIT | PWM_OUT_1_BIT ), true);

	// enable output
	GPIOPinTypeGPIOOutput( MOTOR_SHIFTER_OE_PORT, MOTOR_SHIFTER_OE);
	GPIOPinWrite(MOTOR_SHIFTER_OE_PORT,MOTOR_SHIFTER_OE,(unsigned char)~MOTOR_SHIFTER_OE);

	//enable bridge 0
	GPIOPinTypeGPIOOutput(BRIDGE0_EN_PORT,BRIDGE0_EN);
	GPIOPinWrite(BRIDGE0_EN_PORT,BRIDGE0_EN, BRIDGE0_EN);

	//enable bridge 1
	GPIOPinTypeGPIOOutput(BRIDGE1_EN_PORT,BRIDGE1_EN);
	GPIOPinWrite(BRIDGE1_EN_PORT, BRIDGE1_EN, BRIDGE1_EN);

	// fail inputs 0
	GPIOPinTypeGPIOInput(BRIDGE0_FS_PORT,BRIDGE0_FS);
	GPIOPadConfigSet(BRIDGE0_FS_PORT,BRIDGE0_FS,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);

	// fail inputs 1
	GPIOPinTypeGPIOInput(BRIDGE1_FS_PORT,BRIDGE1_FS);
	GPIOPadConfigSet(BRIDGE1_FS_PORT,BRIDGE1_FS,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);


	//************* QEI
	// Povolení hodin periferiím
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC|SYSCTL_PERIPH_GPIOE);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0|SYSCTL_PERIPH_QEI1);

	// Configure pins as QEI inputs
	GPIOPinTypeQEI(QEI0_PORT, QEI0_PHA);
	GPIOPinTypeQEI(QEI0_PORT, QEI0_PHB);
	GPIOPinTypeQEI(QEI1_PORT, QEI1_PHA);
	GPIOPinTypeQEI(QEI1_PORT, QEI1_PHB);

	// QEI configuration - možno upravit update obou hran pro vyšší rozlišení a taky  QEI_CONFIG_SWAP
	QEIConfigure(QEI0_BASE, QEI_CONFIG_CAPTURE_A_B|QEI_CONFIG_NO_RESET|QEI_CONFIG_QUADRATURE|QEI_CONFIG_NO_SWAP, 511);
	QEIConfigure(QEI1_BASE, QEI_CONFIG_CAPTURE_A_B|QEI_CONFIG_NO_RESET|QEI_CONFIG_QUADRATURE|QEI_CONFIG_NO_SWAP, 511);

	QEIVelocityConfigure(QEI0_BASE,QEI_VELDIV_1,qei_period);
	QEIVelocityEnable(QEI0_BASE);
	QEIVelocityConfigure(QEI1_BASE,QEI_VELDIV_1,qei_period);
	QEIVelocityEnable(QEI1_BASE);

	//
	QEIIntEnable(QEI0_BASE,QEI_INTTIMER);// pøerušení od èasovaèe
	QEIIntEnable(QEI1_BASE,QEI_INTTIMER); // QEI_INTDIR|QEI_INTERROR

	QEIEnable(QEI0_BASE);
	QEIEnable(QEI1_BASE);

	IntPrioritySet(INT_QEI0,(5<<5)); // nastavit QEI0 vyšší prioritu s ohledem na MAX_INPERRUPT PRIO
	IntPrioritySet(INT_QEI1,(6<<5));
	IntEnable(INT_QEI0);
	IntEnable(INT_QEI1);


	RegulatorOdometryReset();

	RegulatorReset(&myDrive.mot1.reg);
	RegulatorReset(&myDrive.mot2.reg);

	RegulatorSetPID(&myDrive.mot1.reg, 0.5, 0.19, 1.3);
	RegulatorSetPID(&myDrive.mot2.reg, 0.5, 0.19, 1.3);

	RegulatorSetParams(&myDrive.mot1.reg, 0.9, 0.5, 1.0);
	RegulatorSetParams(&myDrive.mot2.reg, 0.9, 0.5, 1.0);

	RegulatorSetScaleLimit(&myDrive.mot1.reg,(pwm_period/MOTOR_PULSES_PER_VOLT/8.0),pwm_period);
	RegulatorSetScaleLimit(&myDrive.mot2.reg,(pwm_period/MOTOR_PULSES_PER_VOLT/8.0),pwm_period);


	InitADC();

	// vytvoøíme fronty pro senzory a aktory
	xSpeedActQ  = xQueueCreate( 2, ( unsigned portBASE_TYPE ) sizeof( struct SensorActor) );
	xMotorPWMQ1 = xQueueCreate( 1, ( unsigned portBASE_TYPE ) sizeof( short ) );
	xMotorPWMQ2 = xQueueCreate( 1, ( unsigned portBASE_TYPE ) sizeof( short ) );

	//
	short pwm = 0;
	if (xQueueSend(xMotorPWMQ1, &pwm, 10) != pdTRUE)
	{

	}
	if (xQueueSend(xMotorPWMQ2, &pwm, 10) != pdTRUE)
	{

	}

	vSemaphoreCreateBinary( xWaitData );
	if( xWaitData == NULL )
	{

	}
	MotorControlSetState(MOTOR_RUNNING);

	return xTaskCreate(MotorControl_task, (signed portCHAR *) "MOTOR", 256, NULL, priority , NULL);
}

void MotorControlSetState( enum MotorState st)
{
	switch (st)
	{
	case MOTOR_RUNNING:
		GPIOPinTypeGPIOOutput( MOTOR_SHIFTER_OE_PORT, MOTOR_SHIFTER_OE);
		GPIOPinWrite(MOTOR_SHIFTER_OE_PORT,MOTOR_SHIFTER_OE,(unsigned char)~MOTOR_SHIFTER_OE);
		myDrive.state = st;
		break;
	case MOTOR_HARMONIC_BALANCE:
		GPIOPinTypeGPIOOutput( MOTOR_SHIFTER_OE_PORT, MOTOR_SHIFTER_OE);
		GPIOPinWrite(MOTOR_SHIFTER_OE_PORT,MOTOR_SHIFTER_OE,(unsigned char)~MOTOR_SHIFTER_OE);
		myDrive.state = st;
		break;
	case MOTOR_MANUAL:
		GPIOPinTypeGPIOOutput( MOTOR_SHIFTER_OE_PORT, MOTOR_SHIFTER_OE);
		GPIOPinWrite(MOTOR_SHIFTER_OE_PORT,MOTOR_SHIFTER_OE,(unsigned char)~MOTOR_SHIFTER_OE);
		myDrive.state = st;
		break;
	case MOTOR_STOP:
		GPIOPinTypeGPIOOutput( MOTOR_SHIFTER_OE_PORT, MOTOR_SHIFTER_OE);
		GPIOPinWrite(MOTOR_SHIFTER_OE_PORT,MOTOR_SHIFTER_OE,(unsigned char)~MOTOR_SHIFTER_OE);
		myDrive.mot1.reg.desired = 0;
		myDrive.mot2.reg.desired = 0;
		myDrive.state = st;
		break;
	default:
		//state out of range - do nothing or MOTOR_FAILURE???
		break;
	case MOTOR_FAILURE:
	case MOTOR_SHUTDOWN:
		GPIOPinTypeGPIOOutput( MOTOR_SHIFTER_OE_PORT, MOTOR_SHIFTER_OE);
		GPIOPinWrite(MOTOR_SHIFTER_OE_PORT,MOTOR_SHIFTER_OE,(unsigned char)MOTOR_SHIFTER_OE);
		myDrive.mot1.reg.desired = 0;
		myDrive.mot2.reg.desired = 0;
		myDrive.state = st;
		break;
	}
}

signed portBASE_TYPE MotorControlWaitData(portTickType timeout)
{
	return xSemaphoreTake( xWaitData, timeout );
}

void MotorControlSetWheelSpeed(signed short v1, signed short v2)
{
	// saturation??
	//we only transfer input paramtrs into regulator structure
	myDrive.mot1.reg.desired = v1;
	myDrive.mot2.reg.desired = v2;
}

void MotorControlSetSpeed(float v, float w)
{
	myDrive.mot1.reg.desired = (short)( 0.5 / WHEEL_DISTANCE_PEER_QEI_PULSE * ( (2.0*v*(1.0/SPEED_REG_FREQ)) + (w*(1.0/SPEED_REG_FREQ)*WHEEL_DISTANCE) ));
	myDrive.mot2.reg.desired = (short)(-0.5 / WHEEL_DISTANCE_PEER_QEI_PULSE * ( (2.0*v*(1.0/SPEED_REG_FREQ)) - (w*(1.0/SPEED_REG_FREQ)*WHEEL_DISTANCE) ));
}

void RegulatorOdometryReset( void)
{
	myDrive.position.theta = 0.0;
	myDrive.position.x = 0.0;
	myDrive.position.y = 0.0;
}

enum MotorState MotorControlGetState( void)
{
	// we only return actual state
	return myDrive.state;
}


void MotorControl_task( void * param)
{
	struct SensorActor speed;
	short pwm = 0;
	char lastMotor = 0;

	// dummy delay
	vTaskDelay(100);

	while(1)
	{
		if( xQueueReceive(xSpeedActQ, &speed, 100) == pdTRUE )
		{
			xQueueHandle pwmQue;
			struct MotorControl * motor;

			lastMotor = speed.id;

			if (lastMotor == MOTOR_1)
			{
				pwmQue = xMotorPWMQ1;
				motor =  &myDrive.mot1;
			}
			else if (lastMotor == MOTOR_2)
			{
				pwmQue = xMotorPWMQ2;
				motor =  &myDrive.mot2;
			}
			else
			{
				MotorControlSetState(MOTOR_FAILURE);
				SetError(ERROR_MOTOR);
			}

			switch (myDrive.state)
			{
			case MOTOR_MANUAL:
				pwm = RegulatorAction(&motor->reg, speed.value, 1);
				break;
			case MOTOR_RUNNING:
				pwm = RegulatorAction(&motor->reg, speed.value, 0);
				break;
			case MOTOR_HARMONIC_BALANCE:
				motor->reg.measured = speed.value;
				//! test Metoda harmonické rovnováhy
				if (speed.value>0)
				{
					motor->reg.action = pwm = -HARMONIC_BALANCE_RELAY_LIMIT;
				}
				else
				{
					motor->reg.action = pwm = HARMONIC_BALANCE_RELAY_LIMIT;
				}
				break;
			case MOTOR_FAILURE:
			case MOTOR_STOP:
			case MOTOR_SHUTDOWN:
			default://error
				motor->reg.measured = speed.value;
				pwm = 0;
				break;
			}

			if (xQueueSend(pwmQue, &pwm, 100) != pdTRUE)
			{
				MotorControlSetState(MOTOR_FAILURE);
				SetError(ERROR_MOTOR);
			}
		}
		else
		{
			MotorControlSetState(MOTOR_FAILURE);
			SetError(ERROR_MOTOR);
			//timeout
			//fatal error
		}

		if (lastMotor == MOTOR_2)
		{
			float scale;
			float delta_odo;
			float delta_phi;

			MeasureADC();

			//! odometry
#if MOTOR_ENABLE_ODOMETRY == 1
			delta_odo = (float)(  myDrive.mot1.reg.measured - myDrive.mot2.reg.measured ) * WHEEL_DISTANCE_PEER_QEI_PULSE / 2.0;
			delta_phi = (float)( -myDrive.mot1.reg.measured - myDrive.mot2.reg.measured ) * WHEEL_DISTANCE_PEER_QEI_PULSE / WHEEL_DISTANCE ;

			myDrive.position.x +=  delta_odo * cosf(myDrive.position.theta);
			myDrive.position.y +=  delta_odo * sinf(myDrive.position.theta);
			myDrive.position.theta += delta_phi;
#endif

			//! kontrola Fail statusu H-mostù
			if ( GPIOPinRead(BRIDGE0_FS_PORT,BRIDGE0_FS)==0 )
				MotorControlSetState(MOTOR_FAILURE);

			if ( GPIOPinRead(BRIDGE1_FS_PORT,BRIDGE1_FS)==0 )
				MotorControlSetState(MOTOR_FAILURE);

			//! kontrola stavu baterie
			if (myDrive.batt_voltage < (short)VOLTAGE2ADC(BATERRY_MINIMAL_VOLTAGE))
			{
				SetError(ERROR_BATT);
				MotorControlSetState(MOTOR_SHUTDOWN); // pro snížení spotøeby
			}
			else
			{
				scale = (short)pwm_period / (ADC2VOLTAGE(MOTOR_PULSES_PER_VOLT)) / myDrive.batt_voltage;

				if (scale < 2.0)
				{
					RegulatorSetScaleLimit(&myDrive.mot1.reg, scale, pwm_period);
					RegulatorSetScaleLimit(&myDrive.mot2.reg, scale, pwm_period);
				}

				ClearError(ERROR_BATT);// redundant
			}

			if( xSemaphoreGive( xWaitData ) != pdTRUE )
			{
				// there is no data consument
			}
		}
	}
}

void QEI0_IRQHandler( void)
{
	signed portBASE_TYPE xHigherPriorityTaskWoken;
	signed portBASE_TYPE cTaskWoken = pdFALSE;

	struct SensorActor speed;
	short pwm;

#if configUSE_TRACE_FACILITY==1
	portENTER_CRITICAL();
	vTraceStoreISRBegin(2);
	portEXIT_CRITICAL();
#endif
	//! upravit - pøímo registr s maskou
	switch( QEIIntStatus(QEI0_BASE, true) )
	{
	case QEI_INTERROR:
		myDrive.mot1.qei_errors++;
		break;
	case QEI_INTDIR:

		break;
	case QEI_INTTIMER:
		portENTER_CRITICAL();
		{
			//! pøímo registr
			speed.value = QEIVelocityGet(QEI0_BASE);
			//speed.value *= QEIDirectionGet(QEI0_BASE); // alternativa
			if (QEIDirectionGet(QEI0_BASE) < 0) // backward?
				speed.value *= -1;//! šlo by jednodušeji?

			speed.id = MOTOR_1;
		}
		portEXIT_CRITICAL();

		QEIIntClear(QEI0_BASE, QEI_INTDIR|QEI_INTTIMER|QEI_INTERROR);//!registr

		if( xQueueReceiveFromISR( xMotorPWMQ1, &pwm, &cTaskWoken ) == pdTRUE )
		{
			if (pwm == 0) // no signal
			{

				PWMOutputState(PWM_BASE, PWM_OUT_0_BIT|PWM_OUT_1_BIT, false);
			}
			else if (pwm<0) // reverz
			{
				pwm *= -1;
				portENTER_CRITICAL();
				{
					PWMPulseWidthSet(PWM_BASE, PWM_OUT_1, pwm);
					PWMOutputState(PWM_BASE, PWM_OUT_1_BIT, true);
					PWMOutputState(PWM_BASE, PWM_OUT_0_BIT, false);
				}
				portEXIT_CRITICAL();
			}
			else // forward
			{
				portENTER_CRITICAL();
				{
					PWMPulseWidthSet(PWM_BASE, PWM_OUT_0, pwm);
					PWMOutputState(PWM_BASE, PWM_OUT_0_BIT, true);
					PWMOutputState(PWM_BASE, PWM_OUT_1_BIT, false);
				}
				portEXIT_CRITICAL();
			}
		}

		xQueueSendFromISR( xSpeedActQ, &speed, &xHigherPriorityTaskWoken );
		if( xHigherPriorityTaskWoken != pdFALSE )
		{
			taskYIELD();
		}

		break;
	case QEI_INTINDEX:
	default:
		break;
	}

#if configUSE_TRACE_FACILITY==1
	portENTER_CRITICAL();
	vTraceStoreISREnd();
	portEXIT_CRITICAL();
#endif
}

void QEI1_IRQHandler( void)
{
	signed portBASE_TYPE xHigherPriorityTaskWoken;
	signed portBASE_TYPE cTaskWoken = pdFALSE;

	struct SensorActor speed;
	short pwm;

#if configUSE_TRACE_FACILITY==1
	portENTER_CRITICAL();
	vTraceStoreISRBegin(3);
	portEXIT_CRITICAL();
#endif

	switch( QEIIntStatus(QEI1_BASE, true))
	{
	case QEI_INTERROR:
		myDrive.mot2.qei_errors++;
		break;
	case QEI_INTDIR:
		break;
	case QEI_INTTIMER:
		portENTER_CRITICAL();
		{
			speed.value = QEIVelocityGet(QEI1_BASE);
			//speed.value *= QEIDirectionGet(QEI0_BASE); // alternativa
			if (QEIDirectionGet(QEI1_BASE) < 0) // backward?
				speed.value *= -1;

			speed.id = MOTOR_2;
		}
		portEXIT_CRITICAL();

		QEIIntClear(QEI1_BASE, QEI_INTDIR|QEI_INTTIMER|QEI_INTERROR);

		if( xQueueReceiveFromISR( xMotorPWMQ2, &pwm, &cTaskWoken ) == pdTRUE )
		{
			// omezení proti pøetékání
			if (pwm == 0) // no signal
			{
				PWMOutputState(PWM_BASE, PWM_OUT_2_BIT|PWM_OUT_3_BIT, false);
			}
			else if (pwm<0) // reverz
			{
				pwm *= -1;
				portENTER_CRITICAL();
				{
					PWMPulseWidthSet(PWM_BASE, PWM_OUT_3, pwm);
					PWMOutputState(PWM_BASE, PWM_OUT_3_BIT, true);
					PWMOutputState(PWM_BASE, PWM_OUT_2_BIT, false);
				}
				portEXIT_CRITICAL();
			}
			else // forward
			{
				portENTER_CRITICAL();
				{
					PWMPulseWidthSet(PWM_BASE, PWM_OUT_2, pwm);
					PWMOutputState(PWM_BASE, PWM_OUT_2_BIT, true);
					PWMOutputState(PWM_BASE, PWM_OUT_3_BIT, false);
				}
				portEXIT_CRITICAL();
			}
		}

		xQueueSendFromISR( xSpeedActQ, &speed, &xHigherPriorityTaskWoken );
		if( xHigherPriorityTaskWoken != pdFALSE )
		{
			taskYIELD();
		}

		break;
	case QEI_INTINDEX:
	default:
		break;
	}

#if configUSE_TRACE_FACILITY==1
	portENTER_CRITICAL();
	vTraceStoreISREnd();
	portEXIT_CRITICAL();
#endif
}
