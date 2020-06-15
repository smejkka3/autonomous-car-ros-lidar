//////////////////////////////////////////////////////////////////////////////////////
//                           _______________________
//                      \| WILD THUMPER  ROBOT SYSTEM |/
//                            \_-_-_-_-_-_-_-_-_-_/        
// ----------------------------------------------------------------------------
// ------------------- [c]2010 / 2011 - AREXX ENGINEERING ---------------------
// -------------------------- http://www.arexx.com/ ---------------------------
//////////////////////////////////////////////////////////////////////////////////////
// File: WT_UART.h
// Version: 1.0
// Target: 		Wild Thumper - ATMEGA664 @20.00 MHz   (Main and Motor Controller)
// Author(s): 	Bastiaan Scherphof 
// 			  	Hein Wielink 
//////////////////////////////////////////////////////////////////////////////////////
// Description:
//
// The Wild Thumper uart function Library header file. Detailled description
// of each function can be found in the WT_UART.c file!
//
//////////////////////////////////////////////////////////////////////////////////////
// CHANGELOG AND LICENSING INFORMATION CAN BE FOUND AT THE END OF THIS FILE!
//////////////////////////////////////////////////////////////////////////////////////

//includes
#include <avr/pgmspace.h> 	// Program memory (=Flash ROM) access routines.
#include <stdlib.h>			// C standard functions (e.g. itoa...)
#include <string.h>
#include <avr/io.h>			// I/O Port definitions
#include <avr/interrupt.h>	// Interrupt macros (e.g. cli(), sei())


//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	USART Functions																	//
//																					//
//////////////////////////////////////////////////////////////////////////////////////
// Usart1 initalisatie
void USART0_Init(unsigned long BAUD0);
// Uart0 Leesfunctie
extern unsigned char USART0_Read ( void );
// Uart0 zendfunctie
extern void USART0_Write( unsigned char data ); 
// Uart 0 zend int functie
void USART0_WriteInt(int16_t number, uint8_t base);
// Same as USART0_WriteInt, but with defined length.
void USART0_WriteIntLength(int16_t number, uint8_t base, uint8_t length);
// Uart0 zendstring functie
extern void USART0_WriteString(char *S); 
// uart0 function that writes a string of specified length
void USART0_WriteStringLength(char *string, uint8_t length, uint8_t offset);


// Usart1 initalisatie
void USART1_Init(unsigned long BAUD1);
// Uart1 Leesfunctie
extern unsigned char USART1_Read ( void );
// Uart1 zendfunctie
extern void USART1_Write( unsigned char data ); 
// Uart1 zendstring functie
extern void USART1_WriteString(char *S);   

//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	INT to STRING Function													//
//																					//
//////////////////////////////////////////////////////////////////////////////////////
#define HEX 16
#define DEC 10
#define OCT 8
#define BIN 2

//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	RF Protocol Decoder Function													//
//																					//
//////////////////////////////////////////////////////////////////////////////////////

// Declarations for Recieved Data and Protocol 
#define My_Adress 255

#define Protocol_StartByte  		1
#define Protocol_StopByte			4

#define Protocol_Byte_Steering 		'B'
#define Protocol_Byte_Front			'F'
#define Protocol_Byte_Back			'B'
#define Protocol_Byte_Left			'L'
#define Protocol_Byte_Right			'R'

//Protocol Routine 
void RF_Protocol_Decoder(unsigned char ReceivedByte);
void RF_SetReceivedDataHandler(void (*requestedRFDataHandler)(unsigned char, char[32]));

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
