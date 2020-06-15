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
// This is the RP6 Robot Base Library - it contains the following functions:
// - Processor initialisation
// - LED Control
// - A/D Convertor (Motor Current)
// - Encoder reading and speed measurement
// - Motor Control (Automatic speed control and PWM + Direction control
//   + failsafe functions (overcurrent and defect motor/encoder detection))
//
// PLEASE ALSO READ THE WILD THUMPER MANUAL!!! THERE YOU WILL FIND EXAMPLE
// CODE AND SOME FURTHER EXPLANATIONS!
//
// In other parts of this library (WT_UART.c, WT_I2Cmaster.c and
// WT_I2Cslave.c)
// you can find UART communication and I�C Bus communication routines.
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


#include "WildThumperLib_Motor.h"


//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	LED functions / SPI																//
//																					//
//////////////////////////////////////////////////////////////////////////////////////

//This function update the shift register
void updateShiftRegisters(void)
{
	//Configuration SPI
	//SPE =  1 ---> Enable SPI
	//MSTR = 1 ---> Microcontroller is master, Shiftregister is slave
	//SPR0 = 1
	//SPR1 = 0 ---> SCK frequency = fosc / 16 = 20.000.000 / 16 = 1,25 mHz.
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0)|(0<<SPR1);

	// Write byte to shift register, transmission automatically start

	SPDR = Shiftregister_2.byte;
	// Wait for transmission complete
	while(!(SPSR & (1<<SPIF)));

	SPDR = Shiftregister_1.byte;
	// Wait for transmission complete
	while(!(SPSR & (1<<SPIF)));

	//Configuration SPI
	//SPE =  1 ---> Disable SPI
	//MSTR = 1 ---> Microcontroller is slave, Shiftregister is master
	//Set micro controller to slave, else miso is defined as input!!!
	SPCR = (0<<SPE)|(0<<MSTR);

	PORTB  |= MISO_SPI;				// Enable STRB
	asm(" nop");					// Short time delay
	PORTB&= ~MISO_SPI;				// Disable STRB
}


//Led blinking by startup
void StartUp_Ledblinking (void)
{
	for(char i = 0; i<4 ; i++)
	{
		Shiftregister_2.LED7 = 1;
		for(char i = 0; i<4 ; i++)
		{
			mSleep(100);
			updateShiftRegisters();
			Shiftregister_2.byte = (Shiftregister_2.byte<<1);
		}
	}
	Shiftregister_2.byte = 0b00000000;
	mSleep(200);
	Shiftregister_2.byte = 0b11110000;
	updateShiftRegisters();
	mSleep(500);
	Shiftregister_2.byte = 0b00000000;
	updateShiftRegisters();
}


//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	Status / Error																	//
//																					//
//////////////////////////////////////////////////////////////////////////////////////
char Check_Motor_Flags (void)
{
	char Error_Status = 0;

	if (!Error_Motor_1) Error_Status += 1;
	if (!Error_Motor_2) Error_Status += 2;
	if (!Error_Motor_3) Error_Status += 4;
	if (!Error_Motor_4) Error_Status += 8;
	if (!Error_Motor_5) Error_Status += 16;
	if (!Error_Motor_6) Error_Status += 32;
	
	return Error_Status;
}

char Check_Current (void)
{
	char Current_errors = 0;

	if ( GetADCValue_MotorCurrent(5) > 100 ) Current_errors +=1;
	if ( GetADCValue_MotorCurrent(4) > 100 ) Current_errors +=2;
	if ( GetADCValue_MotorCurrent(3) > 100 ) Current_errors +=4;
	if ( GetADCValue_MotorCurrent(2) > 100 ) Current_errors +=8;
	if ( GetADCValue_MotorCurrent(1) > 100 ) Current_errors +=16;
	if ( GetADCValue_MotorCurrent(0) > 100 ) Current_errors +=32;
	
	return Current_errors;
}

// Check the Encoders
char Check_Encoders (void)
{
	char Encoder_Errors = 0;
	
	// If more than 5 times no pulses detected, then..
	if (SIG1_NoPuls > 5 )
	{
		// To Avoid a overflow of this variable
		SIG1_NoPuls = 6;
		// Set the first bit of Encoder_Errors
		Encoder_Errors +=1;
		// Speed of the motor is SetPoint of that motor, because the PID doesn't work without pulses
		Speed1 = SetPoint1;
	}
	if (SIG2_NoPuls > 5 )
	{
		// To Avoid a overflow of this variable
		SIG2_NoPuls = 6;
		// Set the second bit of Encoder_Errors
		Encoder_Errors +=2;
		// Speed of the motor is SetPoint of that motor, because the PID doesn't work without pulses
		Speed2 = SetPoint2;
	}
	if (SIG3_NoPuls > 5 )
	{
		// To Avoid a overflow of this variable
		SIG3_NoPuls = 6;
		// Set the third bit of Encoder_Errors
		Encoder_Errors +=4;
		// Speed of the motor is SetPoint of that motor, because the PID doesn't work without pulses
		Speed3 = SetPoint3;
	}
	if (SIG4_NoPuls > 5 )
	{
		// To Avoid a overflow of this variable
		SIG4_NoPuls = 6;
		// Set the fourth bit of Encoder_Errors
		Encoder_Errors +=8;
		// Speed of the motor is SetPoint of that motor, because the PID doesn't work without pulses
		Speed4 = SetPoint4;
	}
	if (SIG5_NoPuls > 5 )
	{
		// To Avoid a overflow of this variable
		SIG5_NoPuls = 6;
		// Set the fifth bit of Encoder_Errors
		Encoder_Errors +=16;
		// Speed of the motor is SetPoint of that motor, because the PID doesn't work without pulses
		Speed5 = SetPoint5;
	}
	if (SIG6_NoPuls > 5 )
	{
		// To Avoid a overflow of this variable
		SIG6_NoPuls = 6;
		// Set the sixth bit of Encoder_Errors
		Encoder_Errors +=32;
		// Speed of the motor is SetPoint of that motor, because the PID doesn't work without pulses
		Speed6 = SetPoint6;
	}

	if(Encoder_Errors > 0)
	{
		//Shiftregister_2.LED4 = true;
	}
	else
	{
		//Shiftregister_2.LED4 = false;
	}

	// Update the shift registers
	updateShiftRegisters();

	return Encoder_Errors;

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
	ADCSRA = (1<<ADEN) |(1<<ADSC) |(1<<ADATE)|(0<<ADIF)|(0<<ADATE)|(0<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); 	// ADC Enable, ADC Start Conversion, Auto Trigger Enable, Division Factor 8
	ADCSRB = (0<<ADTS2)|(0<<ADTS1)|(0<<ADTS0);															// Free Running mode
}

// Funtion for ADC Conversion
unsigned char ADC_Conversion (unsigned char Channel) //PA0 = Channel 1 ... PA7 = Channel 8
{
	ADCL = 0;	// Reset ADCL
	ADCH = 0;	// Reset ADCH
	ADMUX &= (1<<REFS1)|(1<<REFS0)|(1<<ADLAR)|(0<<MUX4)|(0<<MUX3) |(0<<MUX2) |(0<<MUX1) |(0<<MUX0);		// Reset Channel
	ADMUX |= Channel;	// Set Channel
	
	for(int N=0; N<1000;N++)
	{
		asm("NOP");	// Wait, Assembler instruction : No operation
	}

	ADCSRA |= (1 << ADSC); 			// start Analog to Digital Conversion
	while(!(ADCSRA & (1<<ADIF)));	// Wait for the AD conversion to complete
	ADCSRA |= (1 << ADIF); 			//set the bit to clear ADIF flag
	
	return ADCH;					//Return the measured value
}

// This function return 1 byte BatteryVoltage ADC Value
unsigned char GetADCValue_MotorCurrent (char motor)
{
	return ADC_Conversion(motor);
}

//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	Motor																			//
//																					//
//////////////////////////////////////////////////////////////////////////////////////

void PWM_Init (void)
{
	// Normal port operation, OC0A and OC0B disconnected
	// Mode: CTC  * Top: OCRA  * Update of OCRx at :Immediate  * TOV Flag Set on : MAX
	TCCR0A = (0<<COM0A1)|(0<<COM0A0)|(0<<COM0B1)|(0<<COM0B0)|(1<<WGM01)|(0<<WGM00);
	// clkI/O/8 (From prescalar)
	TCCR0B = (0<<FOC0A)|(0<<FOC0B)|(0<<CS02)|(1<<CS01)|(0<<CS00);
	// We want a maximum pulswidth of approximately 50Hz and a resolution of 1000
	// 1/50Hz=20ms ; Timer Tick = 20ms/resolution = 20us
	// The timer increased every ; 1/0,00002 = 50000
	OCR0A = (F_CPU/8)/50000; //50000
	// Start Timer 0
	TIMSK0 |= (1 << OCIE0A);
}



ISR(TIMER0_COMPA_vect)
{
	// Counter for 0 to Dutye cycle
	static unsigned short Counter=0;
	
	// If the Dute cycle value raised, then restart
	if (Counter>=DUTYCYCLE_PWM)
	{
		// Set counter 0;
		Counter=0;
		// Start all wheels
		WHEEL_PORT_12   |= (1<<WHEEL1)|(1<<WHEEL2);
		WHEEL_PORT_3456 |= (1<<WHEEL3)|(1<<WHEEL4)|(1<<WHEEL5)|(1<<WHEEL6);
	}

	// Counter = Counter + 1
	Counter++;

	if(Counter>=Speed1) WHEEL1_OFF;
	if(Counter>=Speed2) WHEEL2_OFF;
	if(Counter>=Speed3) WHEEL3_OFF;
	if(Counter>=Speed4) WHEEL4_OFF;
	if(Counter>=Speed5) WHEEL5_OFF;
	if(Counter>=Speed6) WHEEL6_OFF;
}

// This function sets the Direction, FORWARD, BACKWARD, TURN and BRAKE
// Speed is one byte 0 is min en 255 is max
// Direction is LEFT RIGHT and NONE
// The angle is the sharpness of the bend
void Drive(unsigned char Direction_FB, unsigned char Speed, unsigned char Direction_LR, unsigned char Angle)
{
	// If forward, set all shiftregisters on forward
	if(Direction_FB==FORWARD)
	{
		Shiftregister_1.FWD1 = true;
		Shiftregister_1.REV1 = false;

		Shiftregister_1.FWD2 = true;
		Shiftregister_1.REV2 = false;

		Shiftregister_1.FWD3 = true;
		Shiftregister_1.REV3 = false;

		Shiftregister_1.FWD4 = true;
		Shiftregister_1.REV4 = false;

		Shiftregister_2.FWD5 = true;
		Shiftregister_2.REV5 = false;

		Shiftregister_2.FWD6 = true;
		Shiftregister_2.REV6 = false;
		//*/DEBUG/*/ USART0_WriteString("FORWARD");
		
	}
	// If backward, set all shift registers on backward
	else if(Direction_FB==BACKWARD)
	{
		Shiftregister_1.FWD1 = false;
		Shiftregister_1.REV1 = true;

		Shiftregister_1.FWD2 = false;
		Shiftregister_1.REV2 = true;

		Shiftregister_1.FWD3 = false;
		Shiftregister_1.REV3 = true;

		Shiftregister_1.FWD4 = false;
		Shiftregister_1.REV4 = true;

		Shiftregister_2.FWD5 = false;
		Shiftregister_2.REV5 = true;

		Shiftregister_2.FWD6 = false;
		Shiftregister_2.REV6 = true;
		//*/DEBUG/*/ USART0_WriteString("BACKWARD");
	}
	// If else, set all shiftregisters on false, thats brake
	else
	{
		Shiftregister_1.FWD1 = false;
		Shiftregister_1.REV1 = false;

		Shiftregister_1.FWD2 = false;
		Shiftregister_1.REV2 = false;

		Shiftregister_1.FWD3 = false;
		Shiftregister_1.REV3 = false;

		Shiftregister_1.FWD4 = false;
		Shiftregister_1.REV4 = false;

		Shiftregister_2.FWD5 = false;
		Shiftregister_2.REV5 = false;

		Shiftregister_2.FWD6 = false;
		Shiftregister_2.REV6 = false;
		//*/DEBUG/*/ USART0_WriteString("BRAKE");
	}

	// If left, slower left side
	if(Direction_LR==LEFT)
	{
		SetPoint4 = Speed; // << 2;
		SetPoint5 = Speed; // << 2;
		SetPoint6 = Speed; // << 2;
		
		if(Angle<126)
		{
			SetPoint1 = Speed-((Speed*Angle)/126); /* << 2)*/ //Angle Between 0 and 255
			SetPoint2 = Speed-((Speed*Angle)/126); /* << 2)*/
			SetPoint3 = Speed-((Speed*Angle)/126); /* << 2)*/
		}
		else if(Angle==126)
		{
			SetPoint1 = 0;
			SetPoint2 = 0;
			SetPoint3 = 0;
		}
		else
		{
			
			Shiftregister_1.FWD1 = !Shiftregister_1.FWD1;
			Shiftregister_1.REV1 = !Shiftregister_1.REV1;

			Shiftregister_1.FWD2 = !Shiftregister_1.FWD2 ;
			Shiftregister_1.REV2 = !Shiftregister_1.REV2;

			Shiftregister_1.FWD3 = !Shiftregister_1.FWD3;
			Shiftregister_1.REV3 = !Shiftregister_1.REV3;
			
			SetPoint1 = (Speed*(Angle-126))/126; /* << 2)*/ //Angle Between 0 and 255
			SetPoint2 = (Speed*(Angle-126))/126; /* << 2)*/
			SetPoint3 = (Speed*(Angle-126))/126; /* << 2)*/
		}

		//*/DEBUG/*/ USART0_WriteString(" LEFT");
	}
	// If right, slower right side
	else if(Direction_LR==RIGHT)
	{
		
		SetPoint1 = Speed; // << 2;
		SetPoint2 = Speed; // << 2;
		SetPoint3 = Speed; // << 2;
		
		if(Angle<126)
		{
			SetPoint4 = Speed-((Speed*Angle)/126); /* << 2)*/ //Angle Between 0 and 255
			SetPoint5 = Speed-((Speed*Angle)/126); /* << 2)*/
			SetPoint6 = Speed-((Speed*Angle)/126); /* << 2)*/
		}
		else if(Angle==126)
		{
			SetPoint4 = 0;
			SetPoint5 = 0;
			SetPoint6 = 0;
		}
		else
		{
			Shiftregister_1.FWD4 = !Shiftregister_1.FWD1;
			Shiftregister_1.REV4 = !Shiftregister_1.REV1;

			Shiftregister_2.FWD5 = !Shiftregister_1.FWD2;
			Shiftregister_2.REV5 = !Shiftregister_1.REV2;

			Shiftregister_2.FWD6 = !Shiftregister_1.FWD3;
			Shiftregister_2.REV6 = !Shiftregister_1.REV3;
			
			SetPoint4 = (Speed*(Angle-126))/126; /* << 2)*/ //Angle Between 126 and 250
			SetPoint5 = (Speed*(Angle-126))/126; /* << 2)*/
			SetPoint6 = (Speed*(Angle-126))/126; /* << 2)*/
		}
		

	}
	// else is straight, all speeds the same
	else
	{
		SetPoint1 = Speed;// << 2;
		SetPoint2 = Speed;// << 2;
		SetPoint3 = Speed;// << 2;
		SetPoint4 = Speed;// << 2;
		SetPoint5 = Speed;// << 2;
		SetPoint6 = Speed;// << 2;
		//*/DEBUG/*/ USART0_WriteString(" STRAIGHT");
	}
	// If turn, the wheels in the middel slower and one side forward and other side backward
	if(Direction_FB==TURN)
	{
		SetPoint1 = Speed;// << 2;
		SetPoint2 = Speed/4;// << 2)/4;
		SetPoint3 = Speed;// << 2;
		SetPoint4 = Speed;// << 2;
		SetPoint5 = Speed/4;// << 2)/4;
		SetPoint6 = Speed;// << 2;
		if(Direction_LR==LEFT)
		{
			Shiftregister_1.FWD1 = true;
			Shiftregister_1.REV1 = false;

			Shiftregister_1.FWD2 = true;
			Shiftregister_1.REV2 = false;

			Shiftregister_1.FWD3 = true;
			Shiftregister_1.REV3 = false;
			
			Shiftregister_1.FWD4 = false;
			Shiftregister_1.REV4 = true;

			Shiftregister_2.FWD5 = false;
			Shiftregister_2.REV5 = true;

			Shiftregister_2.FWD6 = false;
			Shiftregister_2.REV6 = true;
		}
		else if(Direction_LR==RIGHT)
		{
			Shiftregister_1.FWD1 = false;
			Shiftregister_1.REV1 = true;

			Shiftregister_1.FWD2 = false;
			Shiftregister_1.REV2 = true;
			
			Shiftregister_1.FWD3 = false;
			Shiftregister_1.REV3 = true;

			Shiftregister_1.FWD4 = true;
			Shiftregister_1.REV4 = false;

			Shiftregister_2.FWD5 = true;
			Shiftregister_2.REV5 = false;

			Shiftregister_2.FWD6 = true;
			Shiftregister_2.REV6 = false;
		}
		else
		{
			Shiftregister_1.FWD1 = false;
			Shiftregister_1.REV1 = false;

			Shiftregister_1.FWD2 = false;
			Shiftregister_1.REV2 = false;

			Shiftregister_1.FWD3 = false;
			Shiftregister_1.REV3 = false;

			Shiftregister_1.FWD4 = false;
			Shiftregister_1.REV4 = false;

			Shiftregister_2.FWD5 = false;
			Shiftregister_2.REV5 = false;

			Shiftregister_2.FWD6 = false;
			Shiftregister_2.REV6 = false;
		}
		
	}

	// Update the shiftregisters
	updateShiftRegisters();
}


// This function controls each wheel separately and is for test, FORWARD, BACKWARD, and BRAKE
// Speed is one byte 0 is min en 255 is max
// The angle is the sharpness of the bend
void DriveWheel(unsigned char Wheel, unsigned char Direction_FB, unsigned char Speed)
{
	if(Wheel==1)
	{
		Shiftregister_1.FWD1 = false;
		Shiftregister_1.REV1 = false;
		SetPoint1 = Speed;
		if (Direction_FB==FORWARD) Shiftregister_1.FWD1 = true;
		if (Direction_FB==BACKWARD) Shiftregister_1.REV1 = true;
	}
	else if(Wheel==2)
	{
		Shiftregister_1.FWD2 = false;
		Shiftregister_1.REV2 = false;
		SetPoint2 = Speed;
		if (Direction_FB==FORWARD) Shiftregister_1.FWD2 = true;
		if (Direction_FB==BACKWARD) Shiftregister_1.REV2 = true;
	}
	else if(Wheel==3)
	{
		Shiftregister_1.FWD3 = false;
		Shiftregister_1.REV3 = false;
		SetPoint3 = Speed;// << 2;
		if (Direction_FB==FORWARD) Shiftregister_1.FWD3 = true;
		if (Direction_FB==BACKWARD) Shiftregister_1.REV3 = true;
	}
	else if(Wheel==4)
	{
		Shiftregister_1.FWD4 = false;
		Shiftregister_1.REV4 = false;
		SetPoint4 = Speed;// << 2;
		if (Direction_FB==FORWARD) Shiftregister_1.FWD4 = true;
		if (Direction_FB==BACKWARD) Shiftregister_1.REV4 = true;
	}
	else if(Wheel==5)
	{
		Shiftregister_2.FWD5 = false;
		Shiftregister_2.REV5 = false;
		SetPoint5 = Speed;// << 2;
		if (Direction_FB==FORWARD) Shiftregister_2.FWD5 = true;
		if (Direction_FB==BACKWARD) Shiftregister_2.REV5 = true;
	}
	else if(Wheel==6)
	{
		Shiftregister_2.FWD6 = false;
		Shiftregister_2.REV6 = false;
		SetPoint6 = Speed;// << 2;
		if (Direction_FB==FORWARD) Shiftregister_2.FWD6 = true;
		if (Direction_FB==BACKWARD) Shiftregister_2.REV6 = true;
	}
	else
	{
		// Nothing
	}

	updateShiftRegisters();
}






void DriveDifferential(unsigned char Direction, unsigned char Speed_Left, unsigned char Speed_Right)
{
	Shiftregister_1.FWD1 = false;
	Shiftregister_1.REV1 = false;
	Shiftregister_1.FWD2 = false;
	Shiftregister_1.REV2 = false;
	Shiftregister_1.FWD3 = false;
	Shiftregister_1.REV3 = false;
	Shiftregister_1.FWD4 = false;
	Shiftregister_1.REV4 = false;
	Shiftregister_2.FWD5 = false;
	Shiftregister_2.REV5 = false;
	Shiftregister_2.FWD6 = false;
	Shiftregister_2.REV6 = false;
	if ((Direction==52) || (Direction==53))
	{
		Shiftregister_1.FWD1 = true;
		Shiftregister_1.FWD2 = true;
		Shiftregister_1.FWD3 = true;
	}
	if ((Direction==54) || (Direction==55) )
	{
		Shiftregister_1.REV1 = true;
		Shiftregister_1.REV2 = true;
		Shiftregister_1.REV3 = true;
	}
	if ((Direction==52) || (Direction==54))
	{
		Shiftregister_1.FWD4 = true;
		Shiftregister_2.FWD5 = true;
		Shiftregister_2.FWD6 = true;
	}
	if ((Direction==53) || (Direction==55))
	{
		Shiftregister_1.REV4 = true;
		Shiftregister_2.REV5 = true;
		Shiftregister_2.REV6 = true;
	}
	SetPoint1 = Speed_Left;
	SetPoint2 = Speed_Left;
	SetPoint3 = Speed_Left;
	SetPoint4 = Speed_Right;
	SetPoint5 = Speed_Right;
	SetPoint6 = Speed_Right;
	
	
	
	updateShiftRegisters();
}


//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	Encoder and Stopwatch Timer														//
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
	OCR1A = ((F_CPU/8)/10000-1);//set to 10000 tick/sec (0.1ms)
}

// Start timer1
void Timer1_Start(void)
{
	// Output Compare A Match Interrupt Enable
	TIMSK1 = (1<<OCIE1A);  	// Start timer
	// Set Interrupt enable
	sei();
}

// Stop timer1
void Timer1_Stop(void)
{
	// Output Compare A Match Interrupt Disable
	TIMSK1 = (0<<OCIE1A);  	// Stop timer
}

// Variables for stopwatches
volatile uint16_t delay_timer;
volatile uint8_t ms_timer;
volatile stopwatches_t stopwatches;

// Timer1 Compare match Interrupt Service Routine
ISR(TIMER1_COMPA_vect)
{
	////////////////////////
	// Motor Sig Counters //
	////////////////////////

	// Sigcouters for pulse wide of de SIG Hall sensors
	SIG1_Counter++;
	SIG2_Counter++;
	SIG3_Counter++;
	SIG4_Counter++;
	SIG5_Counter++;
	SIG6_Counter++;
	
	// if the counter, is higher then 2000, then set that counter on 0
	if(SIG1_Counter > 20000)//no value received for 20 ms
	{
		SIG1_PID = 0;
		// No puls more above a minimum setpoint
		if (SetPoint1>MINIMUMSP)
		{
			// Count how many pulses are missing
			SIG1_NoPuls++;
		}
	}
	// The same for SIG2
	if(SIG2_Counter > 20000)
	{
		SIG2_PID = 0;
		// No puls more above a minimum setpoint
		if (SetPoint2>MINIMUMSP)
		{
			// Count how many pulses are missing
			SIG2_NoPuls++;
		}
	}
	// The same for SIG3
	if(SIG3_Counter > 20000)
	{
		SIG3_PID = 0;
		// No puls more above a minimum setpoint
		if (SetPoint2>MINIMUMSP)
		{
			// Count how many pulses are missing
			SIG3_NoPuls++;
		}
	}
	// The same for SIG4
	if(SIG4_Counter > 20000)
	{
		SIG4_PID = 0;
		// No puls more above a minimum setpoint
		if (SetPoint4>MINIMUMSP)
		{
			SIG4_NoPuls++;
		}
	}
	// The same for SIG5
	if(SIG5_Counter > 20000)
	{
		SIG5_PID = 0;
		// No puls more above a minimum setpoint
		if (SetPoint5>MINIMUMSP)
		{
			// Count how many pulses are missing
			SIG5_NoPuls++;
		}
	}
	// The same for SIG6
	if(SIG6_Counter > 20000)
	{
		SIG6_PID = 0;
		// No puls more above a minimum setpoint
		if (SetPoint6>MINIMUMSP)
		{
			// Count how many pulses are missing
			SIG6_NoPuls++;
		}
	}

	/////////////////
	// Stopwatches //
	/////////////////
	delay_timer++;

	if(ms_timer++ >= 10) // 10 * 100us = 1ms
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
// Encoder Signals 																	//
//																					//
//////////////////////////////////////////////////////////////////////////////////////

void PCI_Init (void)
{
	PCICR = (1<<PCIE3)|(1<<PCIE2)|(0<<PCIE1)|(0<<PCIE0);	// Pin Change Interrupt Control Register
	PCIFR = (0<<PCIF3)|(0<<PCIF2)|(0<<PCIF1)|(0<<PCIF0);	// Pin Change Interrupt Flag Register

	// Pin Change Mask Register 0
	PCMSK0 = (0<<PCINT7) |(0<<PCINT6) |(0<<PCINT5) |(0<<PCINT4) |(0<<PCINT3) |(0<<PCINT2) |(0<<PCINT1) |(0<<PCINT0);
	// Pin Change Mask Register 1
	PCMSK1 = (0<<PCINT15)|(0<<PCINT14)|(0<<PCINT13)|(0<<PCINT12)|(0<<PCINT11)|(0<<PCINT10)|(0<<PCINT9) |(0<<PCINT8);
	// Pin Change Mask Register 2
	PCMSK2 = (0<<PCINT23)|(1<<PCINT22)|(1<<PCINT21)|(1<<PCINT20)|(1<<PCINT19)|(0<<PCINT18)|(0<<PCINT17)|(0<<PCINT16);
	// Pin Change Mask Register 3
	PCMSK3 = (0<<PCINT31)|(0<<PCINT30)|(0<<PCINT29)|(0<<PCINT28)|(1<<PCINT27)|(1<<PCINT26)|(0<<PCINT25)|(0<<PCINT24);
}


//////////////////////////////////////////////////////////////////////////////////////
//																					//
// Encoder Reading 																	//
//																					//
//////////////////////////////////////////////////////////////////////////////////////

// Interrupt PCINT16..23; SIG1..SIG4
ISR(PCINT2_vect)
{
	if (SIG1 && SIG1_LastState==false) // Rising edge
	{
		//    __    __    __	//
		// __|  |__|  |__|  |__	//
		//   ^     ^     ^		//
		// Set the last state on TRUE, because its now rising edge
		SIG1_LastState = true;
		// SIG1_Value is the value now and the last sigvalue divided by 2
		SIG1_Value = (SIG1_Counter+SIG1_Value)/2;
		// Recalculate this value to PID value
		SIG1_PID = TIME_TO_PID/SIG1_Value;
		if (SIG1_PID >255)
		{
			SIG1_PID = 255;
		}
		// Set the counter for SIG on 0
		SIG1_Counter = 0;
		// Set no puls on 0 (missing pulses)
		SIG1_NoPuls = 0;
	}
	else if (!SIG1 && SIG1_LastState==true) // Falling Edge
	{
		//    __    __    __	//
		// __|  |__|  |__|  |__	//
		//      ^     ^     ^	//
		// Set the last state on FLASE, because its now falling edge
		SIG1_LastState = false;
		// SIG1_Value is the value now and te last sigvalue divided by 2
		SIG1_Value = (SIG1_Counter+SIG1_Value)/2;
		// Recalculate this value to PID value
		SIG1_PID = TIME_TO_PID/SIG1_Value;
		if (SIG1_PID >255)
		{
			SIG1_PID = 255;
		}
		// Set the counter for SIG on 0
		SIG1_Counter = 0;
		// Set no puls on 0 (missing pulses)
		SIG1_NoPuls = 0;
	}






	//	Signal Encoder 2
	//  Check the last state
	if (SIG2 && SIG2_LastState==false) // Rising edge
	{
		//    __    __    __	//
		// __|  |__|  |__|  |__	//
		//   ^     ^     ^		//
		// Set the last state on TRUE, because its now rising edge
		SIG2_LastState = true;

		// SIG1_Value is the value now and te last sigvalue divided by 2
		SIG2_Value = (SIG2_Counter+SIG2_Value)/2;
		// Recalculate this value to PID value
		SIG2_PID = TIME_TO_PID/SIG2_Value;
		if (SIG2_PID >255)
		{
			SIG2_PID = 255;
		}
		// Set the counter for SIG on 0
		SIG2_Counter = 0;
		// Set no puls on 0 (missing pulses)
		SIG2_NoPuls = 0;
	}
	else if (!SIG2 && SIG2_LastState==true) // Falling Edge
	{
		//    __    __    __	//
		// __|  |__|  |__|  |__	//
		//      ^     ^     ^	//
		// Set the last state on FLASE, because its now falling edge
		SIG2_LastState = false;

		// SIG1_Value is the value now and te last sigvalue divided by 2
		SIG2_Value = (SIG2_Counter+SIG2_Value)/2;
		// Recalculate this value to PID value
		SIG2_PID = TIME_TO_PID/SIG2_Value;
		if (SIG2_PID >255)
		{
			SIG2_PID = 255;
		}
		// Set the counter for SIG on 0
		SIG2_Counter = 0;
		// Set no puls on 0 (missing pulses)
		SIG2_NoPuls = 0;
	}

	//	Signal Encoder 3
	//  Check the last state
	if (SIG3 && SIG3_LastState==false) // Rising edge
	{
		//    __    __    __	//
		// __|  |__|  |__|  |__	//
		//   ^     ^     ^		//
		// Set the last state on TRUE, because its now rising edge
		SIG3_LastState = true;

		// SIG1_Value is the value now and te last sigvalue divided by 2
		SIG3_Value = (SIG3_Counter+SIG3_Value)/2;
		// Recalculate this value to PID value
		SIG3_PID = TIME_TO_PID/SIG3_Value;
		if (SIG3_PID >255)
		{
			SIG3_PID = 255;
		}
		if (SIG3_PID < 0)
		{
			SIG3_PID = 0;
		}
		// Set the counter for SIG on 0
		SIG3_Counter = 0;
		// Set no puls on 0 (missing pulses)
		SIG3_NoPuls = 0;
	}
	else if (!SIG3 && SIG3_LastState==true) // Falling Edge
	{
		//    __    __    __	//
		// __|  |__|  |__|  |__	//
		//      ^     ^     ^	//
		// Set the last state on FLASE, because its now falling edge
		SIG3_LastState = false;

		// SIG1_Value is the value now and te last sigvalue divided by 2
		SIG3_Value = (SIG3_Counter+SIG3_Value)/2;
		// Recalculate this value to PID value
		SIG3_PID = TIME_TO_PID/SIG3_Value;
		if (SIG3_PID >255)
		{
			SIG3_PID = 255;
		}
		if (SIG3_PID < 0)
		{
			SIG3_PID = 0;
		}
		// Set the counter for SIG on 0
		SIG3_Counter = 0;
		// Set no puls on 0 (missing pulses)
		SIG3_NoPuls = 0;
	}

	//	Signal Encoder 4
	//  Check the last state
	if (SIG4 && SIG4_LastState==false) // Rising edge
	{
		//    __    __    __	//
		// __|  |__|  |__|  |__	//
		//   ^     ^     ^		//
		// Set the last state on TRUE, because its now rising edge
		SIG4_LastState = true;

		// SIG1_Value is the value now and te last sigvalue divided by 2
		SIG4_Value = (SIG4_Counter+SIG4_Value)/2;
		// Recalculate this value to PID value
		SIG4_PID = TIME_TO_PID/SIG4_Value;
		if (SIG4_PID >255)
		{
			SIG4_PID = 255;
		}
		// Set the counter for SIG on 0
		SIG4_Counter = 0;
		// Set no puls on 0 (missing pulses)
		SIG4_NoPuls = 0;
	}
	else if (!SIG4 && SIG4_LastState==true) // Falling Edge
	{
		//    __    __    __	//
		// __|  |__|  |__|  |__	//
		//      ^     ^     ^	//
		// Set the last state on FLASE, because its now falling edge
		SIG4_LastState = false;

		// SIG1_Value is the value now and te last sigvalue divided by 2
		SIG4_Value = (SIG4_Counter+SIG4_Value)/2;
		// Recalculate this value to PID value
		SIG4_PID = TIME_TO_PID/SIG4_Value;
		if (SIG4_PID >255)
		{
			SIG4_PID = 255;
		}
		// Set the counter for SIG on 0
		SIG4_Counter = 0;
		// Set no puls on 0 (missing pulses)
		SIG4_NoPuls = 0;
	}
}


// Interrupt PCINT24..31; SIG5..SIG6
ISR(PCINT3_vect)
{
	//	Signal Encoder 5
	//  Check the last state
	if (SIG5 && SIG5_LastState==false) // Rising edge
	{
		//    __    __    __	//
		// __|  |__|  |__|  |__	//
		//   ^     ^     ^		//
		// Set the last state on TRUE, because its now rising edge
		SIG5_LastState = true;

		// SIG1_Value is the value now and te last sigvalue divided by 2
		SIG5_Value = (SIG5_Counter+SIG5_Value)/2;
		// Recalculate this value to PID value
		SIG5_PID = TIME_TO_PID/SIG5_Value;
		if (SIG5_PID >255)
		{
			SIG5_PID = 255;
		}
		// Set the counter for SIG on 0
		SIG5_Counter = 0;
		// Set no puls on 0 (missing pulses)
		SIG5_NoPuls = 0;
	}
	else if (!SIG5 && SIG5_LastState==true) // Falling Edge
	{
		//    __    __    __	//
		// __|  |__|  |__|  |__	//
		//      ^     ^     ^	//
		// Set the last state on FLASE, because its now falling edge
		SIG5_LastState = false;

		// SIG1_Value is the value now and te last sigvalue divided by 2
		SIG5_Value = (SIG5_Counter+SIG5_Value)/2;
		// Recalculate this value to PID value
		SIG5_PID = TIME_TO_PID/SIG5_Value;
		if (SIG5_PID >255)
		{
			SIG5_PID = 255;
		}
		// Set the counter for SIG on 0
		SIG5_Counter = 0;
		// Set no puls on 0 (missing pulses)
		SIG5_NoPuls = 0;
	}

	//	Signal Encoder 6
	//  Check the last state
	if (SIG6 && SIG6_LastState==false) // Rising edge
	{
		//    __    __    __	//
		// __|  |__|  |__|  |__	//
		//   ^     ^     ^		//
		// Set the last state on TRUE, because its now rising edge
		SIG6_LastState = true;

		// SIG1_Value is the value now and te last sigvalue divided by 2
		SIG6_Value = (SIG6_Counter+SIG6_Value)/2;
		// Recalculate this value to PID value
		SIG6_PID = TIME_TO_PID/SIG6_Value;
		if (SIG6_PID >255)
		{
			SIG6_PID = 255;
		}
		// Set the counter for SIG on 0
		SIG6_Counter = 0;
		// Set no puls on 0 (missing pulses)
		SIG6_NoPuls = 0;
	}
	else if (!SIG6 && SIG6_LastState==true) // Falling Edge
	{
		//    __    __    __	//
		// __|  |__|  |__|  |__	//
		//      ^     ^     ^	//
		// Set the last state on FLASE, because its now falling edge
		SIG6_LastState = false;

		// SIG1_Value is the value now and te last sigvalue divided by 2
		SIG6_Value = (SIG6_Counter+SIG6_Value)/2;
		// Recalculate this value to PID value
		SIG6_PID = TIME_TO_PID/SIG6_Value;
		if (SIG6_PID >255)
		{
			SIG6_PID = 255;
		}
		// Set the counter for SIG on 0
		SIG6_Counter = 0;
		// Set no puls on 0 (missing pulses)
		SIG6_NoPuls = 0;
	}
}







int Speed_Calculator (void)
{
	// Wheel is 39 cm
	// 4 keer een puls per omwendeling
	//				 ______________________
	// Wiel rotatie	|					   |__
	//				   __    __    __    __
	// Pulsen	 	__|	 |__|  |__|  |__|  |
	
	unsigned int ReCalcFactor=0;
	unsigned int SIG_ALL = (SIG1_Value + SIG2_Value+ SIG3_Value + SIG4_Value + SIG5_Value + SIG6_Value)/6; // + SIG2_Value + SIG3_Value + SIG4_Value + SIG5_Value + SIG6_Value)/6;
	// SIG_ALL example : SIG_ALL = 200 ; Time of puls was 20ms

	if (SIG_ALL == 0)
	{
		ReCalcFactor = 0;
	}
	else
	{
		ReCalcFactor = 10000/SIG_ALL;
	}
	// Centimetre per second is:
	unsigned int Speed_Measured = 3.25 * ReCalcFactor;
	
	// Decametre per hour is:
	//Speed_Measured = Speed_Measured * 3.6;
	// Kilometre per hour is:
	//Speed_Measured = Speed_Measured/100;
	
	return Speed_Measured;
}

//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	PID Controllers																	//
//																					//
//////////////////////////////////////////////////////////////////////////////////////

// Initialize the PID Controller
void PID_Init(long P_Factor_Scaled,long I_Factor_Scaled,long D_Factor_Scaled )
{
	// Set the SumError of each Wheel control process control on 0
	SumError1 = 0;
	SumError2 = 0;
	SumError3 = 0;
	SumError4 = 0;
	SumError5 = 0;
	SumError6 = 0;

	// Set the LastProcessValue of each Wheel control process control on 0
	LastProcessValue1 = 0;
	LastProcessValue2 = 0;
	LastProcessValue3 = 0;
	LastProcessValue4 = 0;
	LastProcessValue5 = 0;
	LastProcessValue6 = 0;

	// Set all P I D Factors with the scaled factor
	P_Factor = P_Factor_Scaled;
	I_Factor = I_Factor_Scaled;
	D_Factor = D_Factor_Scaled;
	Error_MAX = INT16_MAX / (P_Factor + 1);				//32768 / (P-Factor + 1)
	SumError_MAX = (INT32_MAX/2) / (I_Factor +1);		//1.073.741.824 / 2 /(Ifactor + 1)
}

void PID_P(int Q)
{
	//	if (Q == 1)
	//	  {
	KP=1.7 *32;
	PID_Init(KP,KI,KD);
	//	  }
	// 	else if (Q == 2)
	// 	  {
	// 		KP=0;
	// 		PID_Init(KP,KI,KD);
	// 	  }
	//
}

void PID_I(int Q)
{
	//	if (Q == 1)
	//	  {
	KI=0.09 *32;
	PID_Init(KP,KI,KD);
	//	  }
	// 	else if (Q == 2)
	// 	  {
	// 		KI=0;
	// 		PID_Init(KP,KI,KD);
	//	  }

}

void PID_D(int Q)
{
	//	if (Q == 1)
	//	  {
	KD=0.5 *32;
	PID_Init(KP,KI,KD);
	//	  }
	// 	else if (Q == 2)
	// 	  {
	// 		KD=0;
	// 		PID_Init(KP,KI,KD);
	// 	  }

}

// PID Process
int PID_Controller(char Wheel, int SetPoint, int ProcessValue)
{
	// Create variables of each P I D, Retrun value, process value and errors;
	int P_Term, D_Term, ReturnValue;
	long int Error_Temp, I_Term, Error;
	long int SumError=0;
	long int LastProcessValue=0;
	
	// Check which wheel is selected
	// Copy the SumError and LastProcessValue to this function
	if (Wheel==1)
	{
		SumError = SumError1;
		LastProcessValue = LastProcessValue1;
	}
	else if (Wheel==2)
	{
		SumError = SumError2;
		LastProcessValue = LastProcessValue2;
	}
	else if (Wheel==3)
	{
		SumError = SumError3;
		LastProcessValue = LastProcessValue3;
	}
	else if (Wheel==4)
	{
		SumError = SumError4;
		LastProcessValue = LastProcessValue4;
	}
	else if (Wheel==5)
	{
		SumError = SumError5;
		LastProcessValue = LastProcessValue5;
	}
	else if (Wheel==6)
	{
		SumError = SumError6;
		LastProcessValue = LastProcessValue6;
	}

	// Calculate the Error
	Error = SetPoint - ProcessValue;

	// Calculate the P-term and limit Error overflow
	if(Error > Error_MAX)
	{
		P_Term = P_Factor * Error_MAX;
	}
	else if(Error < -Error_MAX)
	{
		P_Term = P_Factor * -Error_MAX;
	}
	else
	{
		P_Term = P_Factor * Error;
	}

	// Calculate the I-term and limit integral runaway
	Error_Temp = SumError + Error;

	if(Error_Temp > SumError_MAX)
	{
		I_Term = INT32_MAX/2;
		SumError = SumError_MAX;
	}
	else if(Error_Temp < -SumError_MAX)
	{
		I_Term = -INT32_MAX/2;
		SumError = -SumError_MAX;
	}
	else
	{
		SumError = Error_Temp;
		I_Term = I_Factor * SumError;
	}


	// Calculate the D-term
	D_Term = D_Factor * (LastProcessValue - ProcessValue);

	LastProcessValue = ProcessValue;

	// Check wich weel is selected
	// Copy the SumError and LastProcessValue back to wheel SumError and LastProcessValue
	if (Wheel==1)
	{
		SumError1 = SumError;
		LastProcessValue1 = LastProcessValue;
	}
	else if (Wheel==2)
	{
		SumError2 = SumError;
		LastProcessValue2 = LastProcessValue;
	}
	else if (Wheel==3)
	{
		SumError3 = SumError;
		LastProcessValue3 = LastProcessValue;
	}
	else if (Wheel==4)
	{
		SumError4 = SumError;
		LastProcessValue4 = LastProcessValue;
	}
	else if (Wheel==5)
	{
		SumError5 = SumError;
		LastProcessValue5 = LastProcessValue;
	}
	else if (Wheel==6)
	{
		SumError6 = SumError;
		LastProcessValue6 = LastProcessValue;
	}

	// Return all Terms and divided it by the Scaling factor (32)
	ReturnValue = (P_Term + I_Term + D_Term)/PID_SCALING_FACTOR;


	// If the return value higher is then 1000, then return 1000
	if(ReturnValue > 255)
	{
		ReturnValue = 255;
	}
	// If the returnvalue lower is then -1000, then return 0
	else if(ReturnValue < -255)
	{
		ReturnValue = 0;
	}

	ReturnValue = (ReturnValue+255)/2;
	// if
	if (SetPoint < 20)
	{
		ReturnValue = 0;
	}

	// Return the value
	//return (ReturnValue+1000)/2;
	return ReturnValue;
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


volatile uint16_t delay_timer;
volatile uint8_t ms_timer;
volatile stopwatches_t stopwatches;


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


