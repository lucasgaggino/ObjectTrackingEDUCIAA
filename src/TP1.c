

/*==================[inclusions]=============================================*/


#include "TP1.h"       // <= sAPI header
#include <time.h>

/*==================[macros and definitions]=================================*/


#define TICKRATE_MS 50
#define LED_TOGGLE_MS 1000
/*==================[internal data declaration]==============================*/
unsigned int delayer=0;

static const uint8_t CMD_PREFIX = 0xAA;
static const uint8_t CMD_SYNC = 0x0D;
static const uint8_t CMD_ACK = 0x0E;
static const uint8_t CMD_NAK = 0x0F;
static const uint8_t CMD_INITIAL = 0x01;
static const uint8_t CMD_DATA = 0x0A;
static const uint8_t CMD_RESET = 0x08;
static const uint8_t CMD_POWEROFF = 0x09;
static const uint8_t CMD_BAUDRATE = 0x07;
static const uint8_t CMD_PACKAGESIZE = 0x06;
static const uint8_t CMD_SNAPSHOT = 0x05;
static const uint8_t CMD_GETPICTURE = 0x04;
static const uint8_t CMD_LIGHTFREQ = 0x13;

static const uint16_t LAST_JPEG_ACK = 0xF0F0;
static const uint8_t RAW_ACK = 0x0A;
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

void createCommand( const uint8_t cmd, uint8_t param1, uint8_t param2, uint8_t param3, uint8_t param4,struct CAMARA* cam )
{

	(*cam)._command[0] = CMD_PREFIX;
	(*cam)._command[1] = cmd;
	(*cam)._command[2] = param1;
	(*cam)._command[3] = param2;
	(*cam)._command[4] = param3;
	(*cam)._command[5] = param4;


}

void sendCommand(struct CAMARA * cam)
{
  uint8_t i;

  for( i = 0; i < CMD_SIZE; i++ )
  {
	  uartWriteByte( (*cam).SerialChanel, (*cam)._command[i] );
  }


}

bool waitForResponse( uint32_t timeout, uint8_t buffer[], uint16_t bufferLength ,struct CAMARA * cam)
{
  uint8_t byteCnt = 0;
 int time=0;

  while(  time <= timeout*10000 )
  {
    while( uartRxReady(  (*cam).SerialChanel ) )
    {
      buffer[byteCnt] =  uartRxRead( (*cam).SerialChanel);
      byteCnt++;

      if( byteCnt == bufferLength )
      {
        return true;
      }
    }
    time++;
  }

  if( byteCnt > 0 )
  {
    return true;
  }

  return false;
}

bool waitForACK( uint32_t timeout, uint8_t cmdId, struct CAMARA * cam)
{
  bool success = waitForResponse( timeout, (*cam)._receive_cmd, CMD_SIZE, cam );

  // TODO: We are ignoring NAKs here. Should we do something for this
  // specific case?
  if( success && (*cam)._receive_cmd[1] == CMD_ACK && (*cam)._receive_cmd[2] == cmdId )
  {
    return true;
  }

  return false;
}




bool sync(struct CAMARA * cam)
{
  uint8_t attempts = 0;
  bool success;

  // Create the sync command
  createCommand( CMD_SYNC, 0, 0, 0, 0 , cam);

  while( attempts < MAX_SYNC_ATTEMPTS )
  {
    // Send a SYNC command
    sendCommand(cam);

    // Wait for ACK response
    success = waitForACK( RESPONSE_DELAY, CMD_SYNC, cam);

    // Make sure it is an ACK
    if( success )
    {
      // Now wait for a SYNC
      success = waitForResponse( RESPONSE_DELAY,(*cam)._receive_cmd, CMD_SIZE, cam );
      if( success && (*cam)._receive_cmd[1] == CMD_SYNC )
      {
        // All good, flush the buffer
        //_serial.flush();

        // Now send an ACK
        createCommand( CMD_ACK, CMD_SYNC, 0, 0, 0,cam );
        sendCommand(cam);

        return true;
      }
    }

    attempts++;
  }

  return false;
}

bool initial( struct CAMARA * cam)
{
  createCommand( CMD_INITIAL, 0, (*cam).ColorType, (*cam).PreviewResolution, (*cam).JPEGResolution ,cam);
  sendCommand(cam);

  if( waitForACK( RESPONSE_DELAY, CMD_INITIAL, cam ) )
  {
    return true;
  }

  return false;
}

bool setLightFrequency( struct CAMARA * cam )
{
  createCommand( CMD_LIGHTFREQ, (uint8_t) ((*cam).FrequencyType) , 0, 0, 0 ,cam);
  sendCommand(cam);

  if( waitForACK( RESPONSE_DELAY, CMD_LIGHTFREQ ,cam) )
  {
    return true;
  }

  return false;
}

bool setPackageSize( uint16_t size, struct CAMARA * cam )
{
  createCommand( CMD_PACKAGESIZE, 0x08, (uint8_t)(size & 0xFF), (uint8_t)(size >> 8), 0 ,cam);
  sendCommand(cam);

  if( waitForACK( RESPONSE_DELAY, CMD_PACKAGESIZE , cam) )
  {
    // Store package size in instance for future reference
    (*cam)._packageSize = size;
    return true;
  }

  return false;
}

bool snapshot(  uint16_t skipFrames,  struct CAMARA * cam)
{
  createCommand( CMD_SNAPSHOT, (*cam).SnapshotType, (byte)(skipFrames & 0xFF), (byte)(skipFrames >> 8), 0 ,cam);
  sendCommand(cam);

  if( waitForACK( RESPONSE_DELAY, CMD_SNAPSHOT, cam ) )
  {
    return true;
  }

  return false;
}

bool C328reset( bool completeReset, struct CAMARA * cam )
{
  createCommand( CMD_RESET, completeReset ? 0x00 : 0x01, 0, 0, 0xFF, cam );
  sendCommand(cam);

  if( waitForACK( RESPONSE_DELAY, CMD_RESET, cam) )
  {
    return true;
  }

  return false;
}

bool setBaudRate( struct CAMARA * cam )
{
  createCommand( CMD_BAUDRATE, (byte) ((*cam).BaudRate), 0x01, 0, 0 ,cam);
  sendCommand(cam);

  if( waitForACK( RESPONSE_DELAY, CMD_BAUDRATE , cam) )
  {
    return true;
  }

  return false;
}

/*
bool getPicture(  uint16_t processDelay, uint16_t * pictureSize , struct CAMARA * cam )
{
  pictureSize = 0;

  createCommand( CMD_GETPICTURE, (*cam).PictureType, 0, 0, 0, cam );
  sendCommand(cam);

  // Give the camera some time for processing
  delay( processDelay );

  if( !waitForACK( processDelay, CMD_GETPICTURE,cam ) )
    return false;

  if( waitForResponse( processDelay ) && (*cam)._receive_cmd[1] == CMD_DATA )
  {
    // Set the picture size for future reference
    pictureSize = (*cam)._receive_cmd[5] << 8;
    pictureSize |= (*cam)._receive_cmd[4] << 8;
    pictureSize |= (*cam)._receive_cmd[3];

    return true;
  }

  return false;
}
*/

void sendACK( const byte cmd, uint16_t packageId,struct CAMARA * cam )
{
  createCommand( CMD_ACK, cmd, 0, (byte)(packageId & 0xFF), (byte)(packageId >> 8) , cam);
  sendCommand(cam);
}
/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/





int main(void){

   /* ------------- INICIALIZACIONES ------------- */

	struct CAMARA cam;
	cam.SerialChanel=UART_232;
	cam.BaudRate=BAUD115200;
	cam.ColorType=CT_JPEG;
	cam.PreviewResolution=PR_160x120;
	cam.JPEGResolution=JR_640x480;


 /* Inicializar la placa */
 boardConfig();

 /* Inicializar UART_USB a 115200 baudios */
 gpioWrite( LED1, ON );
 uartConfig( UART_232, 115200 );
 if(sync(&cam)) gpioWrite( LEDG, ON );
 else 	 gpioWrite( LEDR, ON );


 delay(2000);
 gpioWrite( LEDG, OFF );
 gpioWrite( LEDR, OFF );

 gpioWrite( LED2, ON );


 //uartWriteString( UART_USB, "DEBUG c/sAPI\r\n" );

 /* ------------- MAIN LOOP ------------- */
 while(1) {
	 if(initial(&cam)) {gpioWrite( LEDG, ON );gpioWrite( LEDR, OFF);}
	 else 	{ gpioWrite( LEDR, ON );gpioWrite( LEDG, OFF);}

	 delay(400);


 }



 return 0 ;
 }



