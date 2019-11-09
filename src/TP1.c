

/*==================[inclusions]=============================================*/


#include "TP1.h"       // <= sAPI header


/*==================[macros and definitions]=================================*/


#define TICKRATE_MS 50
#define LED_TOGGLE_MS 1000
/*==================[internal data declaration]==============================*/
unsigned int delayer=0;
/*==================[internal functions declaration]=========================*/
void myTickHook( void *ptr ){
	if(delayer==9){
	 // modificacion para disminuir la frecuencia 10 vecs
		static bool_t ledState = OFF;
		gpioMap_t led = (gpioMap_t)ptr;


		if( ledState ){
			ledState = OFF;
		}
		else{
			ledState = ON;
		}
		gpioWrite( led, ledState );
		delayer=0;
	}
	else{delayer++;}

 }

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/





int main(void){

   /* ------------- INICIALIZACIONES ------------- */

 /* Inicializar la placa */
 boardConfig();

 /* Inicializar UART_USB a 115200 baudios */
 uartConfig( UART_USB, 115200 );

 uartWriteString( UART_USB, "DEBUG c/sAPI\r\n" );

 /* ------------- MAIN LOOP ------------- */
 while(1) {

    /* Prendo el led azul */
    gpioWrite( LEDB, ON );
    uartWriteString( UART_USB, "LED Toggle\r\n" ); //digo que prendi el led
    delay(500);

    /* Apago el led azul */
    gpioWrite( LEDB, OFF );
    uartWriteString( UART_USB, "LED Toggle\n" ); //digo que apague el led
    delay(500);

 }



 return 0 ;
 }



