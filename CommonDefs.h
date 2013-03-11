#ifndef  __COMMON_DEFS__
#define  __COMMON_DEFS__

#define LED_RED			(GPIO_PIN_2)
#define LED_RED_PORT	(GPIO_PORTF_BASE)

struct Position
{
	double x;
	double y;
	double fi;
};

#endif//__COMMON_DEFS__
