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
#include "sapi_uart.h"                   // Use UART peripherals
#include "sapi_delay.h"
#include "inttypes.h"


#endif




/*==================[inclusions]=============================================*/

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/
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

typedef uint8_t byte;
/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
}
#endif

