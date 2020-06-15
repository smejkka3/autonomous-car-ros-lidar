//////////////////////////////////////////////////////////////////////////////////////
//                           _______________________
//                      \| WILD THUMPER  ROBOT SYSTEM |/
//                            \_-_-_-_-_-_-_-_-_-_/         >>> MAIN CONTROLLER
// ----------------------------------------------------------------------------
// ------------------- [c]2010 / 2011 - AREXX ENGINEERING ---------------------
// -------------------------- http://www.arexx.com/ ---------------------------
//////////////////////////////////////////////////////////////////////////////////////
// File: WildThumperLib_Main.h
// Version: 1.0
// Target: Wild Thumper Main Controller - ATMEGA664 @20.00 MHz
// Author(s): 	Bastiaan Scherphof 
// 			  	Hein Wielink 
//////////////////////////////////////////////////////////////////////////////////////
// Description:
//
//////////////////////////////////////////////////////////////////////////////////////
// CHANGELOG AND LICENSING INFORMATION CAN BE FOUND AT THE END OF THIS FILE!
//////////////////////////////////////////////////////////////////////////////////////

// Includes:
#include <avr/io.h>         	// General I/O Declarations
#include <avr/pgmspace.h>   	// tabel in flash
#include <avr/interrupt.h>  	// ext,timer int sei(), cli()
#include <compat/deprecated.h>  // stay compatible outp, sbi, cbi ed.
#include <avr/eeprom.h>
#include <inttypes.h>
#include <stdbool.h> 
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <compat/twi.h>

#include "WT_UART.h"
#include "WT_Config_Main.h"



//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	LED functions / SPI																//
//																					//
//////////////////////////////////////////////////////////////////////////////////////

// A shadow register that simplifies usage of status LEDs:
union {
 	uint8_t byte;
	struct {
		unsigned LED8:1;
		unsigned LED9:1;
		unsigned LED10:1;
		unsigned LED11:1;
		unsigned LED12:1;
		unsigned LED13:1;
		unsigned Q7:1;
		unsigned Q8:1;
	};
} statusLEDs;


union {
 	uint8_t byte;
	struct {
		unsigned BUMPER_R:1; //1
		unsigned BUMPER_L:1; //2
		unsigned ACS_R:1;	 //4
		unsigned ACS_L:1;	 //8
		unsigned ACS_RF:1;	 //16
		unsigned ACS_LF:1;	 //32
		unsigned RESERE:2;	 //64

	};
} Sensor;

bool Autonomous;
bool Obstacle_Right;
bool Obstacle_Left; 


void updateStatusLEDs(void);
void StartUp_Ledblinking (void);
 void LedOnOff (uint8_t led);

//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	ADC																				//
//																					//
//////////////////////////////////////////////////////////////////////////////////////

#define AREF 				4096 				// Define Reference voltate in millivolts
#define ARESOLUTION 		256					// Define ADC Resolution = 8 bit = 256
#define MVperBIT			AREF/ARESOLUTION	// Define number of millivolts per Bit = reference voltate / resolution   
#define VOLTAGE_DEVIDER		3					// Define the voltage devider ratio for Battery measurement = 1 : 3


void ADC_Init (void);
unsigned char ADC_Conversion (unsigned char Channel);
unsigned char GetADCValue_BatteryVoltage (void);
unsigned char GetADCValue_Temprature (void);
unsigned char GetADCValue_LDR1 (void);
unsigned char GetADCValue_LDR2 (void);

//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	ACS																				//
//																					//
//////////////////////////////////////////////////////////////////////////////////////

#define ACS_L_SET 		PORTC |= (1<<PC7)
#define ACS_L_CLEAR 	PORTC &= ~(1<<PC7) 

#define ACS_LF_SET 		PORTC |= (1<<PC6)
#define ACS_LF_CLEAR 	PORTC &= ~(1<<PC6) 

#define ACS_R_SET 		PORTD |= (1<<PD6)
#define ACS_R_CLEAR 	PORTD &= ~(1<<PD6) 

#define ACS_RF_SET 		PORTD |= (1<<PD5)
#define ACS_RF_CLEAR 	PORTD &= ~(1<<PD5) 

volatile uint8_t acs_event_counter;
uint16_t acs_detect_timeout;

void initACS (void);
char ACS_Check_Left(void);
char ACS_Check_Front_Left(void);
char ACS_Check_Right(void);
char ACS_Check_Front_Right(void);

//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	Pin Change Interrupt															//
//																					//
//////////////////////////////////////////////////////////////////////////////////////

#define BUMPER_SWITCH1		(!(PINB & (1<<PB0)))
#define BUMPER_SWITCH2		(!(PINB & (1<<PB1))) 


char get_bumper_right (void);
char get_bumper_left (void);

void I2C_setInterruptEventHandler(void (*I2C_InterruptHandler)());

//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	MOTOR																			//
//																					//
//////////////////////////////////////////////////////////////////////////////////////

#define	NO_CHANGE		3
#define	FORWARD			2
#define BACKWARD 		1
#define BRAKE			0

//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	Stopwatches																		//
//																					//
//////////////////////////////////////////////////////////////////////////////////////
//Init Timer 1
void Timer1_Init(void);

// Start timer1
extern void Timer1_Start(void);
// Stop timer1
//extern void Timer1_Stop(void);

#define STOPWATCH1 1
#define STOPWATCH2 2
#define STOPWATCH3 4
#define STOPWATCH4 8
#define STOPWATCH5 16
#define STOPWATCH6 32
#define STOPWATCH7 64
#define STOPWATCH8 128


// All are 16bit Stopwatches --> maximum value is 65.535 (16Bit) or ~65.535 seconds!
typedef struct {
	volatile uint8_t watches;
	volatile uint16_t watch1;
	volatile uint16_t watch2;
	volatile uint16_t watch3;
	volatile uint16_t watch4;
	volatile uint16_t watch5;
	volatile uint16_t watch6;
	volatile uint16_t watch7;
	volatile uint16_t watch8;
} stopwatches_t;  
extern volatile stopwatches_t stopwatches;

// Stop functions
#define stopStopwatch1() stopwatches.watches &= ~STOPWATCH1
#define stopStopwatch2() stopwatches.watches &= ~STOPWATCH2
#define stopStopwatch3() stopwatches.watches &= ~STOPWATCH3
#define stopStopwatch4() stopwatches.watches &= ~STOPWATCH4
#define stopStopwatch5() stopwatches.watches &= ~STOPWATCH5
#define stopStopwatch6() stopwatches.watches &= ~STOPWATCH6
#define stopStopwatch7() stopwatches.watches &= ~STOPWATCH7
#define stopStopwatch8() stopwatches.watches &= ~STOPWATCH8

// Start functions
#define startStopwatch1() stopwatches.watches |= STOPWATCH1
#define startStopwatch2() stopwatches.watches |= STOPWATCH2
#define startStopwatch3() stopwatches.watches |= STOPWATCH3
#define startStopwatch4() stopwatches.watches |= STOPWATCH4
#define startStopwatch5() stopwatches.watches |= STOPWATCH5
#define startStopwatch6() stopwatches.watches |= STOPWATCH6
#define startStopwatch7() stopwatches.watches |= STOPWATCH7
#define startStopwatch8() stopwatches.watches |= STOPWATCH8

// Running check functions
#define isStopwatch1Running() (stopwatches.watches & STOPWATCH1)
#define isStopwatch2Running() (stopwatches.watches & STOPWATCH2)
#define isStopwatch3Running() (stopwatches.watches & STOPWATCH3)
#define isStopwatch4Running() (stopwatches.watches & STOPWATCH4)
#define isStopwatch5Running() (stopwatches.watches & STOPWATCH5)
#define isStopwatch6Running() (stopwatches.watches & STOPWATCH6)
#define isStopwatch7Running() (stopwatches.watches & STOPWATCH7)
#define isStopwatch8Running() (stopwatches.watches & STOPWATCH8)

// Get value funtions
#define getStopwatch1() stopwatches.watch1
#define getStopwatch2() stopwatches.watch2
#define getStopwatch3() stopwatches.watch3
#define getStopwatch4() stopwatches.watch4
#define getStopwatch5() stopwatches.watch5
#define getStopwatch6() stopwatches.watch6
#define getStopwatch7() stopwatches.watch7
#define getStopwatch8() stopwatches.watch8

// Set value functions
#define setStopwatch1(__VALUE__) stopwatches.watch1 = __VALUE__
#define setStopwatch2(__VALUE__) stopwatches.watch2 = __VALUE__
#define setStopwatch3(__VALUE__) stopwatches.watch3 = __VALUE__
#define setStopwatch4(__VALUE__) stopwatches.watch4 = __VALUE__
#define setStopwatch5(__VALUE__) stopwatches.watch5 = __VALUE__
#define setStopwatch6(__VALUE__) stopwatches.watch6 = __VALUE__
#define setStopwatch7(__VALUE__) stopwatches.watch7 = __VALUE__
#define setStopwatch8(__VALUE__) stopwatches.watch8 = __VALUE__

//SPI
void writeSPI(uint8_t data);
#define nop() asm volatile("nop\n\t")

//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	Sleep / Delay																	//
//																					//
//////////////////////////////////////////////////////////////////////////////////////

void sleep(uint8_t time);
void mSleep(uint16_t time);

//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	Init																			//
//																					//
//////////////////////////////////////////////////////////////////////////////////////
void Wild_Thumper_Init_All (void);

//////////////////////////////////////////////////////////////////////////////////////
// Additional info
//////////////////////////////////////////////////////////////////////////////////////
// Changelog:
// - v. 1.0 (initial release) 25.11.2010 by Bastiaan Scherphof and Hein Wielink 
//
//////////////////////////////////////////////////////////////////////////////////////
// Bugs, feedback, questions and modifications can be posted on the AREXX Forum
// on http://www.arexx.com/forum/ !
// Of course you can also write us an e-mail to: info@arexx.nl
// AREXX Engineering may publish updates from time to time on AREXX.com!
//////////////////////////////////////////////////////////////////////////////////////
// - LICENSE -
// GNU GPL v2 (http://www.gnu.org/licenses/gpl.txt, a local copy can be found
// on the Wild Thumper CD in the Wild Thumper source code folders!)
// This program is free software. You can redistribute it and/or modify
// it under the terms of the GNU General Public License version 2 as published
// by the Free Software Foundation.
//////////////////////////////////////////////////////////////////////////////////////
// EOF
