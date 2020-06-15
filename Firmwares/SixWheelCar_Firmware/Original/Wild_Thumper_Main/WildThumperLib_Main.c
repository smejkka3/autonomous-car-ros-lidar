//////////////////////////////////////////////////////////////////////////////////////
//                           _______________________
//                      \| WILD THUMPER  ROBOT SYSTEM |/
//                            \_-_-_-_-_-_-_-_-_-_/         >>> MAIN CONTROLLER
// ----------------------------------------------------------------------------
// ------------------- [c]2010 / 2011 - AREXX ENGINEERING ---------------------
// -------------------------- http://www.arexx.com/ ---------------------------
//////////////////////////////////////////////////////////////////////////////////////
// File: 		WildThumperLib_Main.c
// Version: 	1.0
// Target: 		Wild Thumper Main Controller - ATMEGA664 @20.00 MHz
// Author(s): 	Bastiaan Scherphof 
// 			  	Hein Wielink 
//////////////////////////////////////////////////////////////////////////////////////
// Description:
// This is the Wild Thumper Main Controller Base Library - it contains the following functions:
// - Processor initialisation
// - LED Control
// - Bumpers
// - A/D Convertor (Motor Current, Light Sensors, Battery Voltage...)
// - Encoder reading and speed measurement 
// - Motor Control (Automatic speed control and PWM + Direction control
//   + failsafe functions (overcurrent and defect motor/encoder detection))
// - ACS (Anti Collision System) 
// - Timing functions (Delays, Stopwatches etc.)
//
// PLEASE ALSO READ THE WILD THUMPER MANUAL!!! THERE YOU WILL FIND EXAMPLE
// CODE AND SOME FURTHER EXPLANATIONS!
//
// In other parts of this library (WT_UART.c, WT_I2Cmaster.c and 
// WT_I2Cslave.c)
// you can find UART communication and I²C Bus communication routines.
// -----
// Hint: You should better leave all this as it is if you just started with
// C programming, but it is a very good idea to read the comments and review 
// the code, it will help you to understand C programming for AVR better!
// -----
//
// For the experienced users: 
// This code works OK, but it is not optimal! There is a lot potential for 
// tuning! 
// Well, this leaves some tasks for you and this is what makes most 
// fun: To improve the excisting!  
//
// Of course you are free to add new functions and improvements to this
// library and make them available to the public on the Internet e.g. on
// our Forum!
// Please use the changelog at the end of this file to document your
// changes! And add your name to any new function or modification you added! 
// E.g. a "modified by <name> at <date>" is always a good idea to show 
// other users where and WHAT you changed in the source code!
//
// It is a good idea to make your own includeable libraries instead of
// changing this library - of course only if this is possible.
//
// Or create your own complete library with all specific functions you need.
// This code is GPL'd - s. license at the end of this file!
//
//////////////////////////////////////////////////////////////////////////////////////
// CHANGELOG AND LICENSING INFORMATION CAN BE FOUND AT THE END OF THIS FILE!
//////////////////////////////////////////////////////////////////////////////////////

// Includes:
#include "WildThumperLib_Main.h"

static bool Bumper1_WasPressed = false;
static bool Bumper2_WasPressed = false;

//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	LED functions / SPI																//
//																					//
//////////////////////////////////////////////////////////////////////////////////////

//This function update the shiftregister 
void updateStatusLEDs(void)
  {
	//Configuration SPI
	//SPE =  1 ---> Enable SPI
	//MSTR = 1 ---> Microcontroller is master, Shiftregister is slave 
	//SPR0 = 1 
	//SPR1 = 0 ---> SCK frequency = fosc / 16 = 20.000.000 / 16 = 1,25 MHz.  
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0)|(1<<SPR1);

	// Write byte to shift register, transmission automatically start 
	SPDR = statusLEDs.byte;			
	
	// Wait for transmission complete 	
	while(!(SPSR & (1<<SPIF))); 

	//Configuration SPI
	//SPE =  1 ---> Disable SPI
	//MSTR = 1 ---> Microcontroller is slave, Shiftregister is master
	//Set microcontroller to slave, else miso is defined as input!!!
	SPCR = (0<<SPE)|(0<<MSTR);
	PORTB  |= 01000000;				// pull up sck 
	PORTB  |= MISO;					// Enable STRB 	
	
	for(int N = 0; N<300 ; N++)		//for loop (3 times)
	{asm(" nop");}					// Short time delay 


	PORTB  &= ~MISO;				// Disable STRB
	
  }


//Led blinking by startup
void StartUp_Ledblinking (void)
  {
	for(char i = 0; i<3 ; i++)		//for loop (3 times)
	  {
		statusLEDs.LED8 = 1;		//Set the first bit of the struct statusLeds (Led8 = on)
		for(char i = 0; i<6 ; i++)	//For loop (6 times)
		  {	
			mSleep(100);			//delay 100ms 
			updateStatusLEDs();		//update de leds (write to shift register)
			statusLEDs.byte = (statusLEDs.byte<<1);	//Shift the bit to left (Led9 = on, Led 8 = off)
		  }
	  }

	statusLEDs.byte = 0b00111111;	//All leds on 		
	updateStatusLEDs();				//update de leds (write to shift register)
	mSleep(500);					//delay 500ms
	statusLEDs.byte = 0b00000000;   //All leds off 
	updateStatusLEDs();				//Update de leds (write to shift register)
 }

void LedOnOff (uint8_t led)
	{
		statusLEDs.byte = (uint8_t) led; 	//read data as byte
		updateStatusLEDs();					//Update de leds (write to shift register)
	//	mSleep(2000);					//wait for 2 sec else the leds are not visible. 
	//	statusLEDs.byte = 0b00000000;		//All leds off
	//	updateStatusLEDs();					//Update de leds (write to shift register)
	}

//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	ADC																				//
//																					//
//////////////////////////////////////////////////////////////////////////////////////

// Initialize ADC in 8bit mode
void ADC_Init (void)
  {
	ADMUX  = (0<<REFS1)|(0<<REFS0)|(1<<ADLAR)|(0<<MUX4)|(0<<MUX3) |(0<<MUX2) |(0<<MUX1) |(0<<MUX0);		// AVCC with external capacitor at AREF pin
	ADCSRA = (1<<ADEN) |(1<<ADSC) |(1<<ADATE)|(0<<ADIF)|(0<<ADATE)|(1<<ADPS2)|(0<<ADPS1)|(0<<ADPS0); 	// ADC Enable, ADC Start Conversion, Auto Trigger Enable, Division Factor 8
	ADCSRB = (0<<ADTS2)|(0<<ADTS1)|(0<<ADTS0);															// Free Running mode
  }

// Funtion for ADC Conversion
unsigned char ADC_Conversion (unsigned char Channel) //PA0 = Channel 1 ... PA7 = Channel 8
  {
  	ADCL = 0;	// Reset ADCL
	ADCH = 0;	// Reset ADCH
	ADMUX &= (0<<REFS1)|(0<<REFS0)|(1<<ADLAR)|(0<<MUX4)|(0<<MUX3) |(0<<MUX2) |(0<<MUX1) |(0<<MUX0);		// Reset Channel  
	ADMUX |= Channel;	// Set Channel 
	
	// Wait, Assembler instruction : No operation 
	for(int N = 0; N<300 ; N++)		//for loop (300 times)
	{asm(" nop");}					// Short time delay 
	
	ADCSRA |= (1 << ADSC); 			// start Analog to Digital Conversion 
	while(!(ADCSRA & (1<<ADIF)));	// Wait for the AD conversion to complete
	ADCSRA |= (1 << ADIF); 			//set the bit to clear ADIF flag 
	
	return ADCH;					//Return the measured value 
  }

// This function retrun 1 byte BatteryVoltage ADC Value
unsigned char GetADCValue_BatteryVoltage (void)
  {
	return ADC_Conversion(0);
  }

// This function retrun 1 byte Temprature ADC Value
unsigned char GetADCValue_Temprature (void)
  {
	return ADC_Conversion(3) * 1.6;
  }

// This function retrun 1 byte LDR1 ADC Value
unsigned char GetADCValue_LDR1 (void)
  {
	return ADC_Conversion(1);
  }

// This function retrun 1 byte LDR2 ADC Value
unsigned char GetADCValue_LDR2 (void)
  {
	return ADC_Conversion(2);
  }

//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	ACS																				//
//																					//
//////////////////////////////////////////////////////////////////////////////////////
void initACS (void)
  {
	  // Initialize Timer 0 -  PWM ACS
	  TCCR0A =   (0 << WGM00)  | (1 << WGM01) 		//CTC MODE 
    		  |  (1 << COM0A0) | (0 << COM0A1)		//OC2A OFF
			  |  (1 << COM0B0) | (0 << COM0B1);		//OC2B OFF 
	
	  TCCR0B = (0 << WGM02) |  (0 << CS02)  | (1 << CS01) | (0 << CS00);	//PRESCALER 1 
			
  	  OCR0A  = 34;	//	20000000 / (2*8*(221+1)) =  36036 Hz = 36kHz     (Page.146 datasheet)
	  OCR0B  = 34;	//  20000000 / (2*8*(221+1)) =  36036 Hz = 36kHz     (Page.146 datasheet)
  } 

char ACS_Check_Left(void) 
  {
	acs_detect_timeout=0;
	acs_event_counter=0;
	ACS_L_CLEAR;

	for (char i=0; i<20; i++)
		{	
			
			TCCR0A =   (0 << WGM00) | (1 << WGM01) | (1 << COM0B0) | (0 << COM0B1);
			TCCR0A =   (0 << WGM00) | (1 << WGM01) | (1 << COM0A0) | (0 << COM0A1);
			sleep(20);
			TCCR0A =   (0 << WGM00) | (1 << WGM01) | (0 << COM0B0) | (0 << COM0B1);
			TCCR0A =   (0 << WGM00) | (1 << WGM01) | (1 << COM0A0) | (0 << COM0A1);
			sleep(20);
		}
	while (1==1)
		{
		
		if(acs_event_counter > 3)  // receive min. 4 pulses (object)
			{	
			  
			  statusLEDs.LED11 = true;
			  updateStatusLEDs();	
			  ACS_L_SET;
			  return 1;
			}
		
		else if(acs_detect_timeout++ > 15)  // Timeout (no object)
			{
			  statusLEDs.LED11 = false;
			  updateStatusLEDs();	
			  ACS_L_SET;
			  return 0;
			}
		}
  }

char ACS_Check_Front_Left(void) 
  {
	acs_detect_timeout=0;
	acs_event_counter=0;
	ACS_LF_CLEAR;

	for (char i=0; i<20; i++)
		{
			
			TCCR0A =   (0 << WGM00) | (1 << WGM01) | (1 << COM0B0) | (0 << COM0B1);
			TCCR0A =   (0 << WGM00) | (1 << WGM01) | (1 << COM0A0) | (0 << COM0A1);
			sleep(20);
			TCCR0A =   (0 << WGM00) | (1 << WGM01) | (0 << COM0B0) | (0 << COM0B1);
			TCCR0A =   (0 << WGM00) | (1 << WGM01) | (1 << COM0A0) | (0 << COM0A1);
			sleep(20);
		}
	while (1==1)
		{
		
		if(acs_event_counter > 3)  // receive min. 4 pulses (object)
			{	
			  
			  statusLEDs.LED10 = true;
			  updateStatusLEDs();	
			  ACS_LF_SET;
			  return 1;
			}
		
		else if(acs_detect_timeout++ > 15)  // Timeout (no object)
			{
			  statusLEDs.LED10 = false;
			  updateStatusLEDs();	
			  ACS_LF_SET;
			  return 0;
			}
		}
  }

char ACS_Check_Right(void)
  {
	acs_detect_timeout=0;
	acs_event_counter=0;
	ACS_R_CLEAR;

	for (char i=0; i<20; i++)
		{
			
			TCCR0A =   (0 << WGM00) | (1 << WGM01) | (1 << COM0B0) | (0 << COM0B1);
			TCCR0A =   (0 << WGM00) | (1 << WGM01) | (1 << COM0A0) | (0 << COM0A1);
			sleep(20);
			TCCR0A =   (0 << WGM00) | (1 << WGM01) | (0 << COM0B0) | (0 << COM0B1);//0
			TCCR0A =   (0 << WGM00) | (1 << WGM01) | (1 << COM0A0) | (0 << COM0A1);//1
			sleep(20);
		}
	while (1==1)
		{
		
		if(acs_event_counter > 4)  // receive min. 5 pulses (object)
			{	
			  
			  statusLEDs.LED9 = true;
			  updateStatusLEDs();
			  ACS_R_SET;	
			  return 1;
			}
		
		else if(acs_detect_timeout++ > 15)  // Timeout (no object)
			{
			  statusLEDs.LED9 = false;
			  updateStatusLEDs();	
			  ACS_R_SET;
			  return 0;
			}
		}

  }

char ACS_Check_Front_Right(void)
  {
	acs_detect_timeout=0;
	acs_event_counter=0;
	ACS_RF_CLEAR;

	for (char i=0; i<20; i++)
		{
			
			TCCR0A =   (0 << WGM00) | (1 << WGM01) | (1 << COM0B0) | (0 << COM0B1);
			TCCR0A =   (0 << WGM00) | (1 << WGM01) | (1 << COM0A0) | (0 << COM0A1);
			sleep(20);
			TCCR0A =   (0 << WGM00) | (1 << WGM01) | (0 << COM0B0) | (0 << COM0B1);//0
			TCCR0A =   (0 << WGM00) | (1 << WGM01) | (1 << COM0A0) | (0 << COM0A1);//1
			sleep(20);
		}
	while (1==1)
		{
		
		if(acs_event_counter > 4)  // receive min. 5 pulses (object)
			{	
			  
			  statusLEDs.LED8 = true;
			  updateStatusLEDs();
			  ACS_RF_SET;	
			  return 1;
			}
		
		else if(acs_detect_timeout++ > 15)  // Timeout (no object)
			{
			  statusLEDs.LED8 = false;
			  updateStatusLEDs();	
			  ACS_RF_SET;
			  return 0;
			}
		}

  }

ISR (INT2_vect)
  {
		acs_event_counter++;		// Signal received(+1) 
  }

//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	Pin Change Interrupt															//
//																					//
//////////////////////////////////////////////////////////////////////////////////////

void PCI_Init (void)
  {
	PCICR |= (1<<PCIE3)|(1<<PCIE2)|(1<<PCIE1)|(0<<PCIE0);	// Pin Change Interrupt Control Register 
	PCIFR |= (0<<PCIF3)|(0<<PCIF2)|(0<<PCIF1)|(0<<PCIF0);	// Pin Change Interrupt Flag Register

	// Pin Change Mask Register 0
	PCMSK0 |= (0<<PCINT7) |(0<<PCINT6) |(0<<PCINT5) |(0<<PCINT4) |(0<<PCINT3) |(0<<PCINT2) |(0<<PCINT1) |(0<<PCINT0);
	// Pin Change Mask Register 1
	PCMSK1 |= (0<<PCINT15)|(0<<PCINT14)|(0<<PCINT13)|(0<<PCINT12)|(0<<PCINT11)|(0<<PCINT10)|(1<<PCINT9) |(1<<PCINT8);
	// Pin Change Mask Register 2
	PCMSK2 |= (0<<PCINT23)|(0<<PCINT22)|(0<<PCINT21)|(0<<PCINT20)|(0<<PCINT19)|(0<<PCINT18)|(0<<PCINT17)|(0<<PCINT16);
	// Pin Change Mask Register 3
	PCMSK3 |= (1<<PCINT31)|(0<<PCINT30)|(0<<PCINT29)|(0<<PCINT28)|(0<<PCINT27)|(0<<PCINT26)|(0<<PCINT25)|(0<<PCINT24);
  }


// Interrupt PCINT0..7
ISR(PCINT0_vect)
  {
	acs_event_counter++; 
  }


char get_bumper_left (void) 
  {
  	if (BUMPER_SWITCH2)
	 {  return 1;}
	else 
	{ return 0;}

  }

char get_bumper_right (void)
  {
   	if (BUMPER_SWITCH1)
	 {  return 1;}
	else 
	{ return 0;}
  }


// Interrupt PCINT8..15
INTERRUPT(PCINT1_vect)
  {
 	if(BUMPER_SWITCH1) 	// If bumper switch 1 pressed
	  {	
	  	// Check of Bumper was NOT pressed and if the stopwatch is not running, of the value of stopwatch is more than 20 (contact bounce)
	  	if(!Bumper1_WasPressed&(!(isStopwatch2Running()) | (getStopwatch2()>20)))
		  {
		  	setStopwatch2(0);			// Reset the stopwatch
		  	startStopwatch2();			// Start the stopwatch
			Bumper1_WasPressed = true;	// Set boolean Button was pressed 
			// Progressing......
	 		USART0_WriteString("Bumper Switch 1 Pressed \r\n");	
		  }
	  }
	else // Else...
	  {	
	  	// Check of Bumper WAS pressed and if the stopwatch is not running, of the value of stopwatch is more than 20 (contact bounce)
	  	if(Bumper1_WasPressed&(!(isStopwatch2Running()) | (getStopwatch2()>20)))
		  {
		  	setStopwatch2(0);			// Reset the stopwatch
		  	startStopwatch2();			// Start the stopwatch
			Bumper1_WasPressed = false;	// Set boolean Button was pressed 
			// Progressing...... 
	 		USART0_WriteString("Bumper Switch 1 Unpressed \r\n");
		  }	 
	  }	// END Of BUMPER 1

	if(BUMPER_SWITCH2) 	// If bumper switch 1 pressed
	  {	
	  	// Check of Bumper was NOT pressed and if the stopwatch is not running, of the value of stopwatch is more than 20 (contact bounce)
	  	if(!Bumper2_WasPressed&(!(isStopwatch3Running()) | (getStopwatch3()>20)))
		  {
		  	setStopwatch3(0);			// Reset the stopwatch
		  	startStopwatch3();			// Start the stopwatch
			Bumper2_WasPressed = true;	// Set boolean Button was pressed 
			// Progressing......
	 		USART0_WriteString("Bumper Switch 2 Pressed \r\n");	
		  }
	  }
	else // Else...
	  {	
	  	// Check of Bumper WAS pressed and if the stopwatch is not running, of the value of stopwatch is more than 20 (contact bounce)
	  	if(Bumper2_WasPressed&(!(isStopwatch3Running()) | (getStopwatch3()>20)))
		  {
		  	setStopwatch3(0);			// Reset the stopwatch
		  	startStopwatch3();			// Start the stopwatch
			Bumper2_WasPressed = false;	// Set boolean Button was pressed 
			// Progressing...... 
	 		USART0_WriteString("Bumper Switch 2 Unpressed \r\n");
		  }	 
	  }	// END Of BUMPER 2

  }

// Interrupt PCINT16..23
ISR(PCINT2_vect)
  {

  }

//When an interrupt occurs on PD7, the I2C slave has new data,
//When dis happend, this function handles the interrupt.  
void I2C_InterruptEventHandler_DUMMY(){}
static void (*I2C_InterruptEventHandler)() = I2C_InterruptEventHandler_DUMMY;
void I2C_setInterruptEventHandler(void (*I2C_InterruptHandler)())
  {
	I2C_InterruptEventHandler = I2C_InterruptHandler;
  }


// Interrupt PCINT24..31
ISR(PCINT3_vect)
  {
    // this code will be called anytime that PCINT31 switches 
    // (hi to lo, or lo to hi)
	// Only on the postive flank we want to call the I2C_interrupteventhandler
  	if( PIND & (1<< PIND7) )				//Check if PD7 (PCINT31) is high 
  	  { 
		I2C_InterruptEventHandler();		// Call I2C_InterruptEventHandler 
  	  }
  }


//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	StopWatch																		//
//																					//
//////////////////////////////////////////////////////////////////////////////////////

// Start timer0
void Timer1_Init(void)
  {
  	// No bit set
	TCCR1A = (0<<COM1A1)|(0<<COM1A0)|(0<<COM1B1)|(0<<COM1B0)|(0<<FOC1A)|(0<<FOC1B)|(0<<WGM11)|(0<<WGM10);
	// 8 Prescaler, CTC mode
	TCCR1B = (0<<WGM13)|(1<<WGM12)|(0<<CS12)|(1<<CS11)|(0<<CS10);

	// OCR1A value is FCPU diveded by the prescaler and that dived by ticks per second you want
	OCR1A = ((F_CPU/8)/10000-1);
  }


// Start timer1
void Timer1_Start(void)
  {
  	// Output Compare A Match Interrupt Enable
	TIMSK1 = (1<<OCIE1A);  	// Stop timer
	// Set Interrupt enable
	sei();
  }

// Stop timer1
//void Timer1_Stop(void)
 // {
  	// Output Compare A Match Interrupt Disable
//	TIMSK1 = (0<<OCIE1A);  	// Stop timer
 // }

volatile uint16_t delay_timer;
volatile uint8_t ms_timer;
volatile stopwatches_t stopwatches;
// Timer1 Compare match Interrupt Service Routine
ISR(TIMER1_COMPA_vect)
  {
  	delay_timer++;

	if(ms_timer++ >= 10) // 10 * 100µs = 1ms
	  { 
		// 16bit Stopwatches:
		if(stopwatches.watches & STOPWATCH1)
			stopwatches.watch1++;
		if(stopwatches.watches & STOPWATCH2)
			stopwatches.watch2++;
		if(stopwatches.watches & STOPWATCH3)
			stopwatches.watch3++;
		if(stopwatches.watches & STOPWATCH4)
			stopwatches.watch4++;
		if(stopwatches.watches & STOPWATCH5)
			stopwatches.watch5++;
		if(stopwatches.watches & STOPWATCH6)
			stopwatches.watch6++;
		if(stopwatches.watches & STOPWATCH7)
			stopwatches.watch7++;
		if(stopwatches.watches & STOPWATCH8)
			stopwatches.watch8++;

		ms_timer=0;
	  }
  } 

//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	Sleep / Delay																	//
//																					//
//////////////////////////////////////////////////////////////////////////////////////
//  Delay with the help of timer1.
//  msleep(1) delays for *about* 1ms! Not exaclty, as we do not use assembly routines
//  anywhere in this library!
// 
//  This is a blocking routine, which means that the processor
//  will loop in this routine and (except for interrupts) the
//  normal program flow is stopped!
//  Thus you should use the Stopwatch functions wherever you can!
// 
//  Example:
// 		msleep(1); // delay 1 * 1ms = 1ms 
// 		msleep(10); // delay 10 * 1ms = 10ms 
// 		msleep(100); // delay 100 * 1ms = 100ms 
// 		The maximum delay is:
// 		msleep(65535); // delay 65535 * 1ms = 65535ms = 1 min. 5s     

void sleep(uint8_t time)
  {
	for (delay_timer = 0; delay_timer < time;);
  }


void mSleep(uint16_t time)
  {
	while (time--) sleep(10);
  }


//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	Init																			//
//																					//
//////////////////////////////////////////////////////////////////////////////////////

void WT_Main_Init_All (void)
  {
	portInit();					//Instellen Input Outputs

	USART0_Init(9600);			//Init USART0 

	USART1_Init(9600);			//Init USART1

	APC220_SET_1;				// Enable the SetLine
	
	Timer1_Init();				// Init Timer 1

	ADC_Init();					// ADC Init

	PCI_Init();					// Pin Change Interrupt init

	Timer1_Start();				// Start Timer 1

	sei();						// Interrupt Enable 
  }


//////////////////////////////////////////////////////////////////////////////////////
// Additional info
//////////////////////////////////////////////////////////////////////////////////////
// Changelog:
// - v. 1.0 (initial release) 25.11.2010 by Bastiaan Scherphof and Hein Wielink 
//
// - ACS added
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
