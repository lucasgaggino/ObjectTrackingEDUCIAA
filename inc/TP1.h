/* Copyright 2015-2016, Eric Pernia.
 * All rights reserved.
 *
 * This file is part sAPI library for microcontrollers.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/* Date: 2015-09-23 */

#ifndef _TP1_H_
#define _TP1_H_

#include "sapi_board.h"                  // Use clock peripheral
#include "sapi_tick.h"                   // Use Systick peripheral
#include "sapi_gpio.h"                   // Use GPIO peripherals
#include "sapi_uart.h" // Use UART peripherals
#include "sapi_delay.h"
#include "sapi.h"




#endif




/*==================[inclusions]=============================================*/

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/
//sync Pattern for COM with PC
#define SYNC_PATTERN "xxPleaseSYNCwithMExx"
#define SYNC_PATTERN_SIZE 21
#define SYNC_PATTERN_DEFAULT_TIMEOUT 1000

#define GPU_PACK_DATA_LEN 10
#define GPU_PACK_DATA_SIZE 11
#define DATA_TRANSFER_TIMEOUT 1000

#define PI 3.141592653589793

#define STRING_ANGLES_DELIMITER '@'
// The byte size of commands
#define CMD_SIZE 6

// Number of sync attempts to try before giving up
#define MAX_SYNC_ATTEMPTS 60

// How long to wait for serial communication responses
#define RESPONSE_DELAY 100

// How long to wait for camera to process JPEG data
#define PROCESS_DELAY 1000

// How long to wait between data packages
#define PACKAGE_DELAY 10

// The default size of data packages when retrieving
// JPEG image data
#define DEFAULT_PACKAGE_SIZE 64

// The byte offset where image data starts in a JPEG image
// data package
#define PACKAGE_DATA_START 4

// The byte offset from the end of a data package where
// JPEG image data ends
#define PACKAGE_DATA_END_OFFSET 2

// Maximum allowed errors when reading picture data
#define MAX_ERRORS 15
/*==================[typedef]================================================*/
typedef enum
	    {
	      CT_GRAYSCALE_2 = 0x01,
	      CT_GRAYSCALE_4 = 0x02,
	      CT_GRAYSCALE_8 = 0x03,
	      CT_COLOR_12 = 0x05,
	      CT_COLOR_16 = 0x06,
	      CT_JPEG = 0x07
	    }ColorType_t;
	typedef enum
	    {
	      PR_80x60 = 0x01,
	      PR_160x120 = 0x03
	    }PreviewResolution_t;

	typedef enum
	    {
	      JR_80x64 = 0x01,
	      JR_160x128 = 0x03,
	      JR_320x240 = 0x05,
	      JR_640x480 = 0x07
	    }JPEGResolution_t;

	typedef enum
	    {
	      ST_COMPRESSED = 0x00,
	      ST_UNCOMPRESSED = 0x01
	    }SnapshotType_t;

	typedef enum
	    {
	      PT_SNAPSHOT = 0x01,
	      PT_PREVIEW = 0x02,
	      PT_JPEG = 0x05
	    }PictureType_t;

	typedef enum
	    {
	      FT_50Hz = 0x00,
	      FT_60Hz = 0x01
	    }FrequencyType_t;

	typedef enum
	    {
	      BAUD7200 = 0xFF,
	      BAUD9600 = 0xBF,
	      BAUD14400 = 0x7F,
	      BAUD19200 = 0x5F,
	      BAUD28800 = 0x3F,
	      BAUD38400 = 0x2F,
	      BAUD57600 = 0x1F,
	      BAUD115200 = 0x0F
	    }BaudRate_t;

struct CAMARA {
	    ColorType_t ColorType;
	    PreviewResolution_t PreviewResolution;
	    JPEGResolution_t JPEGResolution;
	    SnapshotType_t SnapshotType;
	    PictureType_t PictureType;
	    FrequencyType_t FrequencyType;
	    BaudRate_t BaudRate;
		uartMap_t SerialChanel;
	    uint16_t _packageSize;
	    uint8_t _command[CMD_SIZE];
	    uint8_t _receive_cmd[CMD_SIZE];
};

struct TargetData {
	float alfa1;//angulo horizontal CAM1
	float beta1;//angulo Vertical CAM1
	float alfa2;
	float beta2;
	float lamda;//distancia al objetivo
	float altura;//alutura en el eje Z con CAM 1 como origen
};

typedef uint8_t byte;


//------------------------TP---SISTEMA DE APUNTADO-----DEFS----------------------------//


#define pi 3.14159265359

#define STEPS_GIRO 450 //pasos para girar 180 grados del motor de la base estaba en 400
#define STEPS_ELEV 750 //lo mismo pero para la otra

#define EN GPIO0 //patita para enable
#define S_ELEV GPIO1 //patita para elevacion
#define S_GIRO GPIO2 //patita para giro
#define DIR_ELEV GPIO3
#define DIR_GIRO GPIO4
#define LIM_GIRO GPIO5
#define LIM_ELEV GPIO6

#define DEM_RETURN 10
#define DEM_WORK 5

#define ENABLE 0 //lo que es el ean¿ble
#define DISABLE 1
#define UP_DIR_GIRO 1 //sentido de giro
#define DOWN_DIR_GIRO 0 //sentido de giro
#define UP_DIR_ELEV 0
#define DOWN_DIR_ELEV 1

#define VTX_P 1
#define	VTY_P 2
#define VTZ_P 3
#define POSX_P 4
#define POSY_P 5
#define POSZ_P 6
#define WINDX_P	7
#define	WINDZ_P 8
#define ATM_P 9
#define CAL_P 10
#define MASS_P 11
#define	CD_P 12
#define VO_P 13
#define PIX_P 14
#define PIY_P 15
#define PIZ_P 16
#define WINDY_P 17

#define PAR_L 17


#define X_I 1
#define	Y_I 2
#define Z_I 3
#define T_I 4
#define XT_I 5
#define YT_I 6
#define ZT_I 7
#define VF_I 8
#define AF_I 9

#define IMP_L 9

#define SERVO_0   SERVO0
#define SERVO_1   SERVO1

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
}
#endif




