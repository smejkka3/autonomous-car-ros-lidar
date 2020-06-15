//////////////////////////////////////////////////////////////////////////////////////
//                           _______________________
//                      \| WILD THUMPER  ROBOT SYSTEM |/
//                            \_-_-_-_-_-_-_-_-_-_/        
// ----------------------------------------------------------------------------
// ------------------- [c]2010 / 2011 - AREXX ENGINEERING ---------------------
// -------------------------- http://www.arexx.com/ ---------------------------
//////////////////////////////////////////////////////////////////////////////////////
// File: 		WT_Config_Motor.c
// Version: 	1.0
// Target: 		Wild Thumper - ATMEGA664 @20.00 MHz   (Motor Controller)
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

#define EF_MOTOR1		(1 << PINA7) //ADC7 (input)
#define EF_MOTOR2		(1 << PINA6) //ADC6 (input)
#define I_MOTOR1		(1 << PINA5) //ADC5 (input)
#define I_MOTOR2		(1 << PINA4) //ADC4 (input)
#define I_MOTOR3		(1 << PINA3) //ADC3 (input)
#define I_MOTOR4		(1 << PINA2) //ADC2 (input) 
#define I_MOTOR5		(1 << PINA1) //ADC1 (input)
#define I_MOTOR6		(1 << PINA0) //ADC0 (input)


// Initial value of port and direction registers.
#define INIT_DDRA 0b00000000
#define INIT_PRTA 0b11000000

// ---------------------------------------------------
// PORTA A/D Convertor channels

//#define EF_MOTOR1			7
//#define EF_MOTOR2			6
#define ADC_MOTOR1			5
#define ADC_MOTOR2			4
#define ADC_MOTOR3			3
#define ADC_MOTOR4			2
#define ADC_MOTOR5			1
#define ADC_MOTOR6			0

// ---------------------------------------------------
// PORTB

#define SR_SCK	 	(1 << PINB7) // Output
#define SR_STR 		(1 << PINB6) // Output
#define SR_DATA		(1 << PINB5) // Output
#define MOTOR1_PWM	(1 << PINB4) // Output
#define MOTOR2_PWM	(1 << PINB3) // Output
#define EF_MOTOR3	(1 << PINB2) // Input
#define EF_MOTOR4  	(1 << PINB1) // Input
#define EF_MOTOR5	(1 << PINB0) // Input

// Initial value of port and direction registers.
#define INIT_DDRB 0b11111000
#define INIT_PRTB 0b00000111

// ---------------------------------------------------
// PORTC

#define EF_MOTOR6	(1 << PINC7) // Input
#define SIG_MOTOR1	(1 << PINC6) // Input
#define SIG_MOTOR2	(1 << PINC5) // Input
#define SIG_MOTOR3	(1 << PINC4) // Input 
#define SIG_MOTOR4	(1 << PINC3) // Input
#define I2C_INT		(1 << PINC2) // Output
#define I2C_SDA		(1 << PINC1) // Output - Input
#define I2C_SCL		(1 << PINC0) // Output - Input

// Initial value of port and direction registers.
#define INIT_DDRC 0b00000100
#define INIT_PRTC 0b10000000

// ---------------------------------------------------
// PORTD

#define MOTOR3_PWM		(1 << PIND7)	//(Input)
#define MOTOR4_PWM		(1 << PIND6)	//(Output)
#define MOTOR5_PWM		(1 << PIND5)	//(Output)
#define MOTOR6_PWM		(1 << PIND4)	//(Output)
#define SIG_MOTOR5		(1 << PIND3)	//(Output)
#define SIG_MOTOR6		(1 << PIND2)	//(Input)
#define TX_MOTOR		(1 << PIND1)	//(Output)
#define RX_MOTOR		(1 << PIND0)	//(Input)

// Initial value of port and direction registers.
#define INIT_DDRD 0b11110010
#define INIT_PRTD 0b00000001

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
