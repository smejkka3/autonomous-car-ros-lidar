//////////////////////////////////////////////////////////////////////////////////////
//                           _______________________
//                      \| WILD THUMPER  ROBOT SYSTEM |/
//                            \_-_-_-_-_-_-_-_-_-_/        
// ----------------------------------------------------------------------------
// ------------------- [c]2010 / 2011 - AREXX ENGINEERING ---------------------
// -------------------------- http://www.arexx.com/ ---------------------------
//////////////////////////////////////////////////////////////////////////////////////
// File: 		WT_Config_Main.c
// Version: 	1.0
// Target: 		Wild Thumper - ATMEGA664 @20.00 MHz   (Main Controller)
// Author(s): 	Bastiaan Scherphof 
// 			  	Hein Wielink 
//////////////////////////////////////////////////////////////////////////////////////
// Description:
// The Wild Thumper Config header file with general definitions. If you don't want
// to include the complete CaterpillarBaseLib because it is too large - then at least
// include this file! 
//
// This file contains helpful definitions that simplify reading the sourcecode.
// Most important are the default settings for Port and Direction registers!
// Hint: You should better leave all this as it is, but it is a good idea to
// read the comments, it will help you to understand C programming for AVR
// better!
//
//////////////////////////////////////////////////////////////////////////////////////

#ifndef WT_CONFIG_H
#define WT_CONFIG_H


//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	In and Output Defines															//
//																					//
//////////////////////////////////////////////////////////////////////////////////////
#define F_CPU			20000000UL   	// 20 Mhz 

/*****************************************************************************/
// I/O PORT pin definitions
// These definitions simplify reading and understanding the source code.
//
// ATTENTION: Initial value of port and direction registers should not
// be changed, if you do not exactly know what you are doing!
//
// Hints for DDRx and PORTx Registers:
// DDRxy = 0 and PORTxy = 0 ==> Input without internal Pullup
// DDRxy = 0 and PORTxy = 1 ==> Input with internal Pullup
// DDRxy = 1 and PORTxy = 0 ==> Output low
// DDRxy = 1 and PORTxy = 1 ==> Output high
// "=1" indicates that the appropriate bit is set.
//
// Example:
// #define INIT_DDRA 0b00010000
// #define INIT_PRTA 0b00000000
//
// This means that ALL ports on PortA are inputs without internal pullups
// except for PortA4, which is an output and initial value is low.
//
// Binary value explanation:
// 0b00010000     = 16 in decimal system
//   ^      ^
// MSB      LSB      (MSB = Most Significant Bit, LSB = Least Significant Bit)
//
// The program should always call the macro "portInit();" FIRST! You can find
// it a bit below. Correct port initialisation is the most important step
// after a hardware reset!

// ---------------------------------------------------
// PORTA

#define BUTTON			(1 << PINA7) //ADC7 (input)
#define Z_ACC			(1 << PINA6) //ADC6 (input)
#define Y_ACC			(1 << PINA5) //ADC5 (input)
#define X_ACC			(1 << PINA4) //ADC4 (input)
#define TEMPERATURE		(1 << PINA3) //ADC3 (input)
#define LDR2			(1 << PINA2) //ADC2 (input) 
#define LDR1			(1 << PINA1) //ADC1 (input)
#define BAT_VOLTAGE		(1 << PINA0) //ADC0 (input)


// Initial value of port and direction registers.
#define INIT_DDRA 0b00000000
#define INIT_PRTA 0b00000000

// ---------------------------------------------------
// PORTA A/D Convertor channels

//#define ADC_BUTTON		7
#define ADC_Z_ACC			6
#define ADC_Y_ACC			5
#define ADC_X_ACC			4
#define ADC_TEMPERATURE		3
#define ADC_LDR2			2
#define ADC_LDR1			1
#define ADC_BAT_VOLTAGE		0

// ---------------------------------------------------
// PORTB

#define SCK	 		(1 << PINB7) // Output
#define MISO 		(1 << PINB6) // Input
#define MOSI 		(1 << PINB5) // Output
#define IR_L 		(1 << PINB4) // Input
#define IR_H		(1 << PINB3) // Input
#define IR_RECEIVER	(1 << PINB2) // Output
#define BUMPER2    	(1 << PINB1) // Output
#define BUMPER1		(1 << PINB0) // Output

// Initial value of port and direction registers.
		    //portB 76543210
#define INIT_DDRB 0b10111011 
#define INIT_PRTB 0b00011011 //old: 00000011

// ---------------------------------------------------
// PORTC

#define ACS__L		(1 << PINC7) // Input
#define ACS__LF		(1 << PINC6) // Input
#define TDI			(1 << PINC5) // I/O (Optional JTAG: TDI)
#define TDO			(1 << PINC4) // I/O (JTAG: TDO)
#define TMS			(1 << PINC3) // I/O (JTAG: TMS)
#define TCK			(1 << PINC2) // I/O (JTAG: TCK)
#define SDA_PC		(1 << PINC1) // Output - Input
#define SCL_PC		(1 << PINC0) // Output - Input

// Initial value of port and direction registers.
		    //portB 76543210
#define INIT_DDRC 0b11000000
#define INIT_PRTC 0b11111100

// ---------------------------------------------------
// PORTD

#define I2C_INT			(1 << PIND7)	//(Input)
#define ACS__R			(1 << PIND6)	//(Output)
#define ACS__RF			(1 << PIND5)	//(Output)
#define SET_APC			(1 << PIND4)	//(Output)
#define TX_APC			(1 << PIND3)	//(Output)
#define RX_APC			(1 << PIND2)	//(Input)
#define TX_MAIN			(1 << PIND1)	//(Output)
#define RX_MAIN			(1 << PIND0)	//(Input)

// Initial value of port and direction registers.
		    //portB 76543210
#define INIT_DDRD 0b01111010
#define INIT_PRTD 0b01110001

//////////////////////////////////////////////////////////////////////////////////////
// I/O Port init macro - always call this first! It is called first from
// initRobotBase() in the CaterpillarBaseLib!
//
// Example:
// int main(void)
// {
// 		portInit();
// 		// ...
//		// your application
//		while(true);
//		return 0;
// }

#define portInit();	\
PORTA = INIT_PRTA;	\
PORTB = INIT_PRTB;	\
PORTC = INIT_PRTC;	\
PORTD = INIT_PRTD;	\
DDRA = INIT_DDRA;	\
DDRB = INIT_DDRB;	\
DDRC = INIT_DDRC;	\
DDRD = INIT_DDRD;

//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	Some additional definitions/macros												//
//																					//
//////////////////////////////////////////////////////////////////////////////////////

#define true 	1
#define false 	0
#define TRUE 	1
#define FALSE 	0

//Defines for the APC220 
#define APC220_SET_1		PORTD |= (1<<PD4) 			//Set APC220 Set line 
#define APC220_SET_0		PORTD &=~ (1<<PD4)			//Clear APC220 Set Line 

#endif

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
