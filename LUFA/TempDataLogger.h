/*
             LUFA Library
     Copyright (C) Dean Camera, 2014.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2014  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaims all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Header file for TempDataLogger.c.
 */

#ifndef _TEMP_DATALOGGER_H_
#define _TEMP_DATALOGGER_H_

	/* Includes: */
		#include <avr/io.h>
		#include <avr/wdt.h>
		#include <avr/power.h>
		#include <avr/interrupt.h>
		#include <stdio.h>
		#include "DataflashManager.h"
		#include "../file.system/ff.h"
//        #include <avr/eeprom.h>


//	#define DUMMY_RTC  //from "AppConfig.h"

	/* Macros: */


// На будущее эти переменные пригодятся для изменяемого периода записи лога !!!
/** Default log interval when the EEPROM is blank, in 500ms ticks. */
//#define DEFAULT_LOG_INTERVAL     10

/** Non-volatile Logging Interval value in EEPROM, stored as a number of 500ms ticks */
//static uint8_t EEMEM LoggingInterval500MS_EEPROM = DEFAULT_LOG_INTERVAL;

/** SRAM Logging Interval value fetched from EEPROM, stored as a number of 500ms ticks */
//static uint8_t LoggingInterval500MS_SRAM;

/** Total number of 500ms logging ticks elapsed since the last log value was recorded */
//static uint16_t CurrentLoggingTicks;




	/* Type Defines: */
// 		typedef struct
// 		{
// 			TimeDate_t TimeDate;
// 			uint8_t    LogInterval500MS;
// 		} Device_Report_t;

	/* Function Prototypes: */
//		void SetupHardware(void);
//		void OpenLogFile(void);
//		void CloseLogFile(void);

#endif

