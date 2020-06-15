//////////////////////////////////////////////////////////////////////////////////////
//                           _______________________
//                      \| WILD THUMPER  ROBOT SYSTEM |/
//                            \_-_-_-_-_-_-_-_-_-_/        
// ----------------------------------------------------------------------------
// ------------------- [c]2010 / 2011 - AREXX ENGINEERING ---------------------
// -------------------------- http://www.arexx.com/ ---------------------------
//////////////////////////////////////////////////////////////////////////////////////
// File: WT_UART.c
// Version: 1.0
// Target: 		Wild Thumper - ATMEGA664 @20.00 MHz   (Main and Motor Controller)
// Author(s): 	Bastiaan Scherphof 
// 			  	Hein Wielink 
//////////////////////////////////////////////////////////////////////////////////////
// Description:
//
// The Wild Thumper UART Library. (UART = "Universal Aynchronous Receiver Transceiver")
//
// Hint: You should better leave all this as it is if you just started with
// C programming, but it is a good idea to read the comments and review the
// code, it will help you to understand C programming for AVR better.
//
// Of course you are free to add new functions and improvements to this
// library and make them available to the public on the Internet.
// Please use the changelog at the end of this file to document your
// changes. And add your name (or nickname) to any new function or 
// modification you added! E.g. a "modified by <nickname> at <date>" is 
// always a good idea to show other users where and what you changed the 
// source code!
//
//////////////////////////////////////////////////////////////////////////////////////
// CHANGELOG AND LICENSING INFORMATION CAN BE FOUND AT THE END OF THIS FILE!
//////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////
// Includes:
#include "WT_UART.h"
#include "WildThumperLib_Main.h"		//Include stopwatches 

//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	Init 																			//
//																					//
//////////////////////////////////////////////////////////////////////////////////////
// Usart0 initialisation 
void USART0_Init(unsigned long BAUD0)
  {
	unsigned long UBRR0SET	=	((F_CPU / (16 * BAUD0)) - 1);
    UCSR0A = 0;
	//Set baud rate 
 	UBRR0H = (UBRR0SET >> 8);
   	UBRR0L = UBRR0SET & 0XFF;			    
	//Enable receiver and transmitter 
	UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);
	//Set frame format: 8data, 1stop bit
	UCSR0C = (0<<USBS0)|(3<<UCSZ00);
  }

// Usart1 initialisation 
void USART1_Init(unsigned long BAUD1)
  {
	unsigned long UBRR1SET	=	((F_CPU / (16 * BAUD1)) - 1);
    UCSR1A = 0;
	//Set baud rate 
 	UBRR1H = (UBRR1SET >> 8);
   	UBRR1L = UBRR1SET & 0XFF;			    
	//Enable receiver and transmitter 
	UCSR1B = (1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1);
	//Set frame format: 8data, 1stop bit
	UCSR1C = (0<<USBS1)|(3<<UCSZ10);
  }


//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	USART 0	Read/Write Functions													//
//																					//
//////////////////////////////////////////////////////////////////////////////////////


// Uart0 Leesfunctie
unsigned char USART0_Read ( void ) 
  {
  	while ( !(UCSR0A &  (_BV(RXC0))) ); 	// Wait until data is received
 	return UDR0;							// Return the received byte
  } 

// Uart0 zendfunctie
void USART0_Write( unsigned char data ) 
  { 
   	while (!(UCSR0A & (1 << UDRE0)));  	// Wait until the transmit buffer is empty
	UDR0 = data;   						// Put data in UDR
  } 

/*
 * Write a number (with specified base) to the UART.
 *
 * Example:
 *
 *			// Write a hexadecimal number to the UART:
 *			USART0_WriteInt(0xAACC,16);
 *			// Instead of 16 you can also write "HEX" as this is defined in the
 *			// RP6RobotBaseLib.h :
 *			USART0_WriteInt(0xAACC, HEX);
 *			// Other Formats:
 *			USART0_WriteInt(1024,DEC);  		// Decimal
 *			USART0_WriteInt(044,OCT);			// Ocal
 *			USART0_WriteInt(0b11010111,BIN);	// Binary
 */

void USART0_WriteInt(int16_t number, uint8_t base)
{
	char buffer[17];
	itoa(number, &buffer[0], base);
	USART0_WriteString(&buffer[0]);
}

/*
 * Same as USART0_WriteInt, but with defined length.
 * This means this routine will add leading zeros to the number if length is
 * larger than the actual value or cut the upper digits if length is smaller
 * than the actual value.
 *
 * Example:
 *
 *			// Write a hexadecimal number to the UART:
 *			USART0_WriteIntLength(0xAACC, 16, 8);
 *			// Instead of 16 you can also write "HEX" as this is defined in the
 *			// RP6RobotBaseLib.h :
 *			USART0_WriteIntLength(0xAACC, HEX, 8);
 *			// Other Formats:
 *			USART0_WriteIntLength(1024,DEC,6);  		// Decimal
 *			USART0_WriteIntLength(044,OCT,4);			// Ocal
 *			USART0_WriteIntLength(0b11010111,BIN,8); 	// Binary
 */
void USART0_WriteIntLength(int16_t number, uint8_t base, uint8_t length)
{
	char buffer[17];
	itoa(number, &buffer[0], base);
	int8_t cnt = length - strlen(buffer);
	if(cnt > 0) {
		for(; cnt > 0; cnt--, USART0_Write('0'));
		USART0_WriteString(&buffer[0]);
	}
	else 
		USART0_WriteStringLength(&buffer[0],length,-cnt);
}

/*
 * Writes a string with specified length and offset from SRAM to UART.
 * If it is a null terminated string, output will be stopped at the
 * end! It does not need to be null terminated, but it is recommended
 * to use only null terminated strings/buffers, otherwise the function could
 * output any SRAM memory data stored after the string until it reaches a 0
 * or the specified length.
 *
 * Example:
 *
 *			writeStringLength("RP6 Robot Sytem\n",16,0);
 *			// would output: "RP6 Robot Sytem\n"
 *			writeStringLength("RP6 Robot Sytem\n",11,4);
 *			// would output: "Robot System"
 * 			writeStringLength("RP6 Robot Sytem\n",40,4);
 *			// would output: "Robot System\n"
 *			// No matter if the specified length is 40 characters!
 *
 */
void USART0_WriteStringLength(char *string, uint8_t length, uint8_t offset)
{
	for(string = &string[offset]; *string && length; length--)
		USART0_Write(*string++);
}

// Uart0 zendstring functie
void USART0_WriteString(char *S)   
  {
   	while (*S)	// Pointer loopt array langs
   	  {
    	while (!(UCSR0A & (1 << UDRE0)));	// Wait until the transmit buffer is empty
  		UDR0 = *S++;						// Put data(character) in UDR
   	  }
  } 

//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	USART 1 Read/Write Functions													//
//																					//
//////////////////////////////////////////////////////////////////////////////////////
// Uart1 Leesfunctie
unsigned char USART1_Read ( void ) 
  {
  	while ( !(UCSR1A &  (_BV(RXC1))) ); 	// Wait until data is received
 	return UDR1;							// Return the received byte
  } 

// Uart1 zendfunctie
void USART1_Write( unsigned char data ) 
  { 
   	while (!(UCSR1A & (1 << UDRE1)));  	// Wait until the transmit buffer is empty
	UDR1 = data;   						// Put data in UDR
  } 

// Uart1 sendstring functie
void USART1_WriteString(char *S)   
  {
   	while (*S)	// Pointer loopt array langs
   	  {
    	while (!(UCSR1A & (1 << UDRE1)));	// Wait until the transmit buffer is empty
  		UDR1 = *S++;						// Put data(character) in UDR
   	  }
  } 

//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	Interrupt routines																//
//																					//
//////////////////////////////////////////////////////////////////////////////////////

// Usart1 Receive Interrupt Service Routine
ISR(USART0_RX_vect)
  {
	//Code for handle data from uart0 
  }


// UART1 is used for the communication between the APC220 and the main controller
// Usart1 Receive Interrupt Service Routine
ISR(USART1_RX_vect)
  {
  	RF_Protocol_Decoder(UDR1);
  } 


//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	RF Protocol Decoder																//
//																					//
//////////////////////////////////////////////////////////////////////////////////////
//RF_ReceivedDataHandler
//void RF_ReceivedDataHandler (unsigned char ControlByte, char *InformationBytes)

void RF_ReceivedDataHandler_DUMMY(unsigned char ControlByte, char *InformationBytes){}
static void (*RF_ReceivedDataHandler)(unsigned char, char[32]) = RF_ReceivedDataHandler_DUMMY;
void RF_SetReceivedDataHandler(void (*requestedRFDataHandler)(unsigned char, char[32]))
  {
	RF_ReceivedDataHandler = requestedRFDataHandler;
  }


// This Function handles the recieved data
void RF_Protocol_Decoder(unsigned char ReceivedByte)
  {
	// This variable keep the current State 
  	static unsigned char State = 0;
	// This variable counts the received bytes
	static unsigned char ReceivedBytes = 0;
	// This variable contains the expected message size  
	static unsigned char BytesToReceive = 0;
	// This variable contains the information size  
	static unsigned char InformationSize = 0;
	// This vabiable contains the ControlByte
	static unsigned char RecievedControlByte = 0;
	// This array contains the information
	static char RecievedInformationBytes[32];
	// This variable keep the current number of information byte
	static unsigned char InformationByteCounter = 0;

	// If last received byte was more than 200ms ago, then TIMEOUT
	if(getStopwatch1() > 200)
	  {
		// Clear all data			
		State = 0;
		ReceivedBytes = 0;
		BytesToReceive = 0;
		InformationSize = 0;
		RecievedControlByte = 0;
		memset(RecievedInformationBytes, 0 ,32);
		InformationByteCounter = 0;
		// Stop stopwatch
		stopStopwatch1();
		// Reset stopwatch
		setStopwatch1(0);
	  }


	switch(State)		
  	  {
  		// State 0 : IDLE
		case 0 :  {
					// If the StartByte received
	  	  			if (ReceivedByte == Protocol_StartByte ) 
				  	  {
				  		// Go to next state
				  		State++;
						// Received = 0 + 1
						ReceivedBytes++;
						// Stat TimeOut stopwatch
						startStopwatch1();
					

					  }
					else
					  {
						State = 0;
						
					  }

					break; // Go to end of case
				  }	
		// State 1 : Check Adress
		case 1 :  {
					// If the ReceivedByte is My_Adress
					if (ReceivedByte == My_Adress)
					  {
				  		// Received + 1
						ReceivedBytes++;
						// Go to the next state
				  		State++;
						  
					  }
					else
					  {
				  		// Otherwise go to the first statement 
						State = 0;
					  }

					// Reset stopwatch
					setStopwatch1(0);			

					break; // Go to end of case
				  }					
		// State 2: Check message size and calculate Information size
		case 2 :  {
					// The RecievedByte in this state contains the size of the message
					BytesToReceive = ReceivedByte;
					// Calculate the size of information bytes : That is BytesToReceive minus start byte, address byte, length byte, control byte and stop byte = 5
					InformationSize = (BytesToReceive - 5);
					// Received + 1
					ReceivedBytes++;
					// Go to the next state
					State++;

					// Reset stopwatch
					setStopwatch1(0);

					break;	// Go to end of case
				  }					
	
		// State 3: Check control byte
		case 3 :  {
					// Put the ReceivedByte in RecievedControlByte
					RecievedControlByte = ReceivedByte;
					// Received + 1
					ReceivedBytes++;
					// Go to the next state
					State++;

					// Reset stopwatch
					setStopwatch1(0);
					
					break;	// Go to end of case
				  }					

		// State 4: 
		case 4 :  {
					// Fill information array with received byte(s)
					RecievedInformationBytes[InformationByteCounter] = ReceivedByte;
					// InformationByteCounter + 1
					InformationByteCounter++;
					// Received + 1
					ReceivedBytes++;					
				
					// If this byte was te last information byte
					if(InformationByteCounter == InformationSize)
					  {
				  		// Go to the next state
				  		State++;
					  }
					
					// Reset stopwatch
					setStopwatch1(0);

					break;	// Go to end of case
				  }
				  
		// State 5: 
		case 5 :  {
					// Received + 1
					ReceivedBytes++;
					// If the stop byte is received and the number of receiverd bytes = expexed bytes, then the hole packet is reveived
					if((ReceivedByte==Protocol_StopByte) & (ReceivedBytes==BytesToReceive))
					  {
						// Process received data
						RF_ReceivedDataHandler(RecievedControlByte,RecievedInformationBytes);

					  }

					// Clear all data			
					State = 0;
					ReceivedBytes = 0;
					BytesToReceive = 0;
					InformationSize = 0;
					RecievedControlByte = 0;
					memset(RecievedInformationBytes, 0 ,32);
					InformationByteCounter = 0;

					// Stop stopwatch
					stopStopwatch1();
					// Reset stopwatch
					setStopwatch1(0);
				
					break;	// Go to end of case
				  }			  				
  	  } //End Switch

  }



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
