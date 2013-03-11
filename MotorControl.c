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
#include "StatusLED.h"
#include "SlipSerial.h"

#define ABS(x) ((x)<0?-x:x)

//BRIDGE 0 => motor L or R ??

#define BRIDGE0_EN				(GPIO_PIN_7)
#define BRIDGE0_EN_PORT			(GPIO_PORTD_BASE)
#define BRIDGE0_IN1				(GPIO_PIN_0)
#define BRIDGE0_IN1_PORT		(GPIO_PORTF_BASE)
#define BRIDGE0_IN2				(GPIO_PIN_1)
#define BRIDGE0_IN2_PORT		(GPIO_PORTG_BASE)
#define BRIDGE0_FS				(GPIO_PIN_4)
#define BRIDGE0_FS_PORT			(GPIO_PORTB_BASE)

#define BRIDGE1_EN				(GPIO_PIN_1)
#define BRIDGE1_EN_PORT			(GPIO_PORTF_BASE)
#define BRIDGE1_IN1				(GPIO_PIN_0)
#define BRIDGE1_IN1_PORT		(GPIO_PORTB_BASE)
#define BRIDGE1_IN2				(GPIO_PIN_1)
#define BRIDGE1_IN2_PORT		(GPIO_PORTB_BASE)
#define BRIDGE1_FS				(GPIO_PIN_5)
#define BRIDGE1_FS_PORT			(GPIO_PORTB_BASE)


#define MOTOR_SHIFTER_OE		(GPIO_PIN_7)
#define MOTOR_SHIFTER_OE_PORT	(GPIO_PORTA_BASE)

#define QEI0_PORT	(GPIO_PORTC_BASE)
#define QEI0_PHA	(GPIO_PIN_4)
#define QEI0_PHB	(GPIO_PIN_6)

#define QEI1_PORT	(GPIO_PORTE_BASE)
#define QEI1_PHA	(GPIO_PIN_3)
#define QEI1_PHB	(GPIO_PIN_2)

static xQueueHandle xSpeedActQ;
static xQueueHandle xMotorPWMQ1;
static xQueueHandle xMotorPWMQ2;

void PWMFault_IRQHandler( void)
{
	//error
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
			speed.value = QEIVelocityGet(QEI0_BASE);
			//speed.value *= QEIDirectionGet(QEI0_BASE); // alternativa
			if (QEIDirectionGet(QEI0_BASE) < 0) // backward?
				speed.value *= -1;

			speed.id = MOTOR_1;
		}
		portEXIT_CRITICAL();

		QEIIntClear(QEI0_BASE, QEI_INTDIR|QEI_INTTIMER|QEI_INTERROR);

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
	//ADCIntEnable(ADC0_BASE,0);
	//IntEnable(INT_ADC0SS0);
	//IntMasterEnable();

	ADCProcessorTrigger(ADC0_BASE, 0);

}

long MeasureADC( void)
{
	unsigned long adcval[3];
	long pocet;

	while(!ADCIntStatus(ADC0_BASE, 0, false)){};

	pocet = ADCSequenceDataGet(ADC0_BASE,0,&adcval[0]);
	ADCProcessorTrigger(ADC0_BASE, 0);

	myDrive.mot1.current_total += myDrive.mot1.current_act = (short) adcval[0];
	myDrive.mot2.current_total += myDrive.mot2.current_act = (short) adcval[1];
	myDrive.batt_voltage = (short) adcval[2];

	return pocet;

}


void MotorControlInit( void)
{
	//Bridges GPIO init
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA|SYSCTL_PERIPH_GPIOB|SYSCTL_PERIPH_GPIOD|SYSCTL_PERIPH_GPIOE|SYSCTL_PERIPH_GPIOF|SYSCTL_PERIPH_GPIOG);

	//*********** PWM
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM);

	// Configure pins as PWM outs
	GPIOPinTypePWM(BRIDGE0_IN1_PORT,BRIDGE0_IN1);
	GPIOPinTypePWM(BRIDGE0_IN2_PORT,BRIDGE0_IN2);

	GPIOPinTypePWM(BRIDGE1_IN1_PORT,BRIDGE1_IN1);
	GPIOPinTypePWM(BRIDGE1_IN2_PORT,BRIDGE1_IN2);

	//konfigurace generátorù - možnosti úprav synchronizace (PWM_GEN_MODE_SYNC | PWM_GEN_MODE_GEN_SYNC_GLOBAL | PWM_GEN_MODE_GEN_SYNC_LOCAL)
	PWMGenConfigure(PWM_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC );
	PWMGenConfigure(PWM_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC );

	// period init
	PWMGenPeriodSet(PWM_BASE, PWM_GEN_0, MOTOR_PWM_PERIOD);
	PWMGenPeriodSet(PWM_BASE, PWM_GEN_1, MOTOR_PWM_PERIOD);

	// widths init
	PWMPulseWidthSet(PWM_BASE, PWM_OUT_0, MOTOR_PWM_PERIOD/10);
	PWMPulseWidthSet(PWM_BASE, PWM_OUT_1, MOTOR_PWM_PERIOD/10);

	PWMPulseWidthSet(PWM_BASE, PWM_OUT_2, MOTOR_PWM_PERIOD/10);
	PWMPulseWidthSet(PWM_BASE, PWM_OUT_3, MOTOR_PWM_PERIOD/10);

	// this enables generators
	PWMGenEnable(PWM_BASE, PWM_GEN_0);
	PWMGenEnable(PWM_BASE, PWM_GEN_1);

	// output matrix
	PWMOutputState(PWM_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT | PWM_OUT_2_BIT | PWM_OUT_3_BIT, false);// dopøedu
	//PWMOutputState(PWM_BASE, PWM_OUT_1_BIT | PWM_OUT_2_BIT, true);// dopøedu
	//PWMOutputState(PWM_BASE, PWM_OUT_0_BIT | PWM_OUT_3_BIT, true);// dopøedu
	//PWMOutputState(PWM_BASE, PWM_OUT_1_BIT | PWM_OUT_2_BIT, false);

	//intver selected
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

	QEIVelocityConfigure(QEI0_BASE,QEI_VELDIV_1,SPEED_REG_PERIOD);
	QEIVelocityEnable(QEI0_BASE);
	QEIVelocityConfigure(QEI1_BASE,QEI_VELDIV_1,SPEED_REG_PERIOD);
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

	/*
	// ************ ADC seq 0//enable triger for ADC
	PWMGenIntTrigEnable(PWM_BASE,PWM_GEN_0,(PWM_TR_CNT_LOAD));

	//PWMIntEnable(PWM_BASE, PWM_INT_GEN_0);
	//IntEnable(INT_PWM0);*/

	myDrive.mot1.speed_des = 0;
	myDrive.mot2.speed_des = 0;
	myDrive.K = 0.9;
	myDrive.Ti = 0.2;

	InitADC();

	// vytvoøíme fronty pro senzory a aktory
	xSpeedActQ  = xQueueCreate( 2, ( unsigned portBASE_TYPE ) sizeof( struct SensorActor) );
	xMotorPWMQ1 = xQueueCreate( 1, ( unsigned portBASE_TYPE ) sizeof( short) );
	xMotorPWMQ2 = xQueueCreate( 1, ( unsigned portBASE_TYPE ) sizeof( short) );

	short pwm = 0;

	if (xQueueSend(xMotorPWMQ1, &pwm, 10) != pdTRUE)
	{

	}
	if (xQueueSend(xMotorPWMQ2, &pwm, 10) != pdTRUE)
	{

	}

	MotorControlSetState(MOTOR_RUNNING);

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
	case MOTOR_MANUAL:
		GPIOPinTypeGPIOOutput( MOTOR_SHIFTER_OE_PORT, MOTOR_SHIFTER_OE);
		GPIOPinWrite(MOTOR_SHIFTER_OE_PORT,MOTOR_SHIFTER_OE,(unsigned char)~MOTOR_SHIFTER_OE);
		myDrive.state = st;
		break;
	case MOTOR_STOP:
		GPIOPinTypeGPIOOutput( MOTOR_SHIFTER_OE_PORT, MOTOR_SHIFTER_OE);
		GPIOPinWrite(MOTOR_SHIFTER_OE_PORT,MOTOR_SHIFTER_OE,(unsigned char)~MOTOR_SHIFTER_OE);
		myDrive.mot1.speed_des = 0;
		myDrive.mot2.speed_des = 0;
		myDrive.state = st;
		break;
	case MOTOR_SHUTDOWN:
		GPIOPinTypeGPIOOutput( MOTOR_SHIFTER_OE_PORT, MOTOR_SHIFTER_OE);
		GPIOPinWrite(MOTOR_SHIFTER_OE_PORT,MOTOR_SHIFTER_OE,(unsigned char)MOTOR_SHIFTER_OE);
		myDrive.mot1.speed_des = 0;
		myDrive.mot2.speed_des = 0;
		myDrive.state = st;
		break;
	default:
		//error?
		break;
	}
}

enum MotorState MotorControlGetState( void)
{
	return myDrive.state;
}

short SpeedReg(struct MotorControl* mc, short measure)
{
	double vystup;

	mc->speed_err = mc->speed_des - measure;
	vystup = mc->sum + (myDrive.K*mc->speed_err);
	mc->sum += myDrive.K * myDrive.Ti * mc->speed_err;

	//omezení integraèní složky
	if (mc->sum  > SUM_LIMIT)
		mc->sum  = SUM_LIMIT;
	if (mc->sum  < -SUM_LIMIT)
		mc->sum  = -SUM_LIMIT;


	//uprava výstupu podle napájecího napìtí
	vystup = vystup * 538.0 / myDrive.batt_voltage;

	if (vystup >  MOTOR_PWM_PERIOD)
		vystup =  MOTOR_PWM_PERIOD;
	if (vystup < -MOTOR_PWM_PERIOD)
		vystup = -MOTOR_PWM_PERIOD;


	return (short) vystup;
}

short regvalues[6];
void MotorControl_task( void * param)
{
	struct SensorActor speed;
	short pwm = 0;
	char lastMotor = 0;

	// Initialization code - create the channel label for a VTracePrintF
	//traceLabel adc_user_event_channel1 = xTraceOpenLabel("ot1");

	//inicialzace HW

	vTaskDelay(100);

	while(1)
	{
		if( xQueueReceive(xSpeedActQ, &speed, 100) == pdTRUE )
		{
			lastMotor = speed.id;

			if (lastMotor == MOTOR_1)
			{
				regvalues[0] = speed.value;

				switch (myDrive.state)
				{
				case MOTOR_RUNNING:
					pwm = SpeedReg(&myDrive.mot1, speed.value);
					break;
				case MOTOR_MANUAL:
					pwm = myDrive.mot1.speed_des;
					break;
				case MOTOR_FAILURE:
				case MOTOR_STOP:
				case MOTOR_SHUTDOWN:
				default://error
					pwm = 0;
					break;
				}

				regvalues[2] = pwm;
				regvalues[4] = myDrive.mot1.speed_err;
				if (xQueueSend(xMotorPWMQ1, &pwm, 10) != pdTRUE)
				{
					MotorControlSetState(MOTOR_FAILURE);
					StatusLEDSetError(ERROR_MOTOR);
				}
			}
			else if (lastMotor == MOTOR_2)
			{
				regvalues[1] = speed.value;

				switch (myDrive.state)
				{
				case MOTOR_RUNNING:
					pwm = SpeedReg(&myDrive.mot2, speed.value);
					break;
				case MOTOR_MANUAL:
					pwm = myDrive.mot2.speed_des;
					break;
				case MOTOR_FAILURE:
				case MOTOR_STOP:
				case MOTOR_SHUTDOWN:
				default://error
					pwm = 0;
					break;
				}

				regvalues[3] = pwm;
				regvalues[5] = myDrive.mot2.speed_err;
				if (xQueueSend(xMotorPWMQ2, &pwm, 10) != pdTRUE)
				{
					MotorControlSetState(MOTOR_FAILURE);
					StatusLEDSetError(ERROR_MOTOR);
				}
			}
			else
			{
				MotorControlSetState(MOTOR_FAILURE);
				StatusLEDSetError(ERROR_MOTOR);
			}
		}
		else
		{
			MotorControlSetState(MOTOR_FAILURE);
			StatusLEDSetError(ERROR_MOTOR);
			//timeout
			//fatal error
		}

		if (lastMotor == MOTOR_2)
		{
			unsigned short values[3];
			char timestamp = 1;
			MeasureADC();

			//èasová znaèka
			SlipSend(ID_TIME_STAMP,(char *) &timestamp, sizeof(char));

			//data regulátorù
			SlipSend(ID_REG,(char *) &regvalues, sizeof(short[6]));

			values[0] = myDrive.mot1.current_act;
			values[1] = myDrive.mot2.current_act;
			values[2] = myDrive.batt_voltage;

			//data ADC mìøení
			SlipSend(ID_ADC,(char *) &values, sizeof(unsigned short[3]));

			SlipSend(ID_MOTOR_MODE, (char *) &myDrive.state, sizeof(enum MotorState) );

			//! kontrola pøipojení motorù

			//! kontrola Fail statusu H-mostù

			//! kontrola stavu baterie
			if (myDrive.batt_voltage < 500)
			{
				StatusLEDSetError(ERROR_BATT);
				MotorControlSetState(MOTOR_SHUTDOWN); // pro maximální snížení spotøeby
			}
			else
				StatusLEDClearError(ERROR_BATT);// dost možná redundantní

		}

		// An advanced user event, at the relevant code location
		//vTracePrintF(adc_user_event_channel1, "otacky 1: %d", speeds[0]);

	}

	//kontrola funkce  - faulty od driverù, errory encodérù, proudy,  pøipojení motorù(pomìr odchylky a otáèek (jejich prùmìrù))

	//vTaskDelete( NULL );

}
