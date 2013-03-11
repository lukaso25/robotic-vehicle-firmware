

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

//! �loha indika�n� led a v�pisu �loh
void StatusLED_task( void * pvParameters );

//! zaznamen�n� chyby
void StatusLEDSetError( enum Err err);

//! zru�en� zobrazen� chyby
void StatusLEDClearError( enum Err err);

