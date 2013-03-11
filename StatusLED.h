

enum Err
{
	ERROR_OK = 1,
	ERROR_BATT = 2,
	ERROR_ACC = 3,
	ERROR_MOTOR = 4,
	ERROR_COMM_SLIP = 5,
	ERROR_COMM_CAN = 6
};
// inicializace GPIO LED
void StatusLEDInit( void);

//! úloha indikaèní led a výpisu úloh
void StatusLED_task( void * pvParameters );

//! zaznamenání chyby
void StatusLEDSetError( enum Err err);

//! zrušení zobrazení chyby
void StatusLEDClearError( enum Err err);

