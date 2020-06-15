//////////////////////////////////////////////////////////////////////////////////////
//                           _______________________
//                      \| WILD THUMPER  ROBOT SYSTEM |/
//                            \_-_-_-_-_-_-_-_-_-_/         >>> MOTOR CONTROLLER
// ----------------------------------------------------------------------------
// ------------------- [c]2010 / 2011 - AREXX ENGINEERING ---------------------
// -------------------------- http://www.arexx.com/ ---------------------------
//////////////////////////////////////////////////////////////////////////////////////
// File: WildThumperLib_Motor.h
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
#include "WT_Config_Motor.h"


//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	In and Output Defines															//
//																					//
//////////////////////////////////////////////////////////////////////////////////////


#define MOSI_SPI (1<<PB5); 
#define MISO_SPI (1<<PB6); 
#define CLK_SPI (1<<PB7); 

//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	LED functions / SPI																//
//																					//
//////////////////////////////////////////////////////////////////////////////////////

// A shadow register that simplifies usage of status LEDs:
union {
 	uint16_t byte;
	struct {
		unsigned FWD1:1;
		unsigned REV1:1;
		unsigned FWD2:1;
		unsigned REV2:1;
		unsigned FWD3:1;
		unsigned REV3:1;
		unsigned FWD4:1;
		unsigned REV4:1;

	};
} Shiftregister_1;



// A shadow register that simplifies usage of status LEDs:
union {
 	uint8_t byte;
	struct {
		unsigned FWD5:1;//1
		unsigned REV5:1;//2
		unsigned FWD6:1;//4
		unsigned REV6:1;//8
		unsigned LED7:1;//16
		unsigned LED6:1;//32
		unsigned LED5:1;//64
		unsigned LED4:1;//128
	};
} Shiftregister_2;


void updateShiftRegisters(void);
void StartUp_Ledblinking (void);


//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	Status / Error																	//
//																					//
//////////////////////////////////////////////////////////////////////////////////////


#define Error_Motor_1		(PINA & EF_MOTOR1) 
#define Error_Motor_2		(PINA & EF_MOTOR2)
#define Error_Motor_3		(PINB & EF_MOTOR3)
#define Error_Motor_4		(PINB & EF_MOTOR4)
#define Error_Motor_5		(PINB & EF_MOTOR5)
#define Error_Motor_6		(PINC & EF_MOTOR6)

char Check_Motor_Flags (void);
char Check_Current (void);
char Check_Encoders (void);


//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	ADC																				//
//																					//
//////////////////////////////////////////////////////////////////////////////////////
void ADC_Init (void);
unsigned char ADC_Conversion (unsigned char Channel);
unsigned char GetADCValue_MotorCurrent (char motor);

//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	MOTOR																			//
//																					//
//////////////////////////////////////////////////////////////////////////////////////

void Drive(unsigned char Direction_FB, unsigned char Speed, unsigned char Direction_LR, unsigned char Angle);
void DriveWheel(unsigned char Wheel, unsigned char Direction_FB, unsigned char Speed);
void DriveDifferential(unsigned char Direction, unsigned char Speed_Left, unsigned char Speed_Right);
void PID_Init(long P_Factor_Scaled,long I_Factor_Scaled,long D_Factor_Scaled );
int PID_Controller(char Wheel, int SetPoint, int ProcessValue);
void PWM_Init (void);

//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	ENCODER																			//
//																					//
//////////////////////////////////////////////////////////////////////////////////////
int Speed_Calculator (void);
void PCI_Init (void);

#define SIG1 	(PINC & (1<<PC6))
#define SIG2 	(PINC & (1<<PC5))
#define SIG3 	(PINC & (1<<PC4))
#define SIG4 	(PINC & (1<<PC3))
#define SIG5 	(PIND & (1<<PD3))
#define SIG6 	(PIND & (1<<PD2))

volatile bool SIG1_LastState;
volatile bool SIG2_LastState;
volatile bool SIG3_LastState;
volatile bool SIG4_LastState;
volatile bool SIG5_LastState;
volatile bool SIG6_LastState;

volatile unsigned short SIG1_Counter;
volatile unsigned short SIG2_Counter;
volatile unsigned short SIG3_Counter;
volatile unsigned short SIG4_Counter;
volatile unsigned short SIG5_Counter;
volatile unsigned short SIG6_Counter;

volatile unsigned short SIG1_Value;

volatile unsigned char SIG1_NoPuls, SIG2_NoPuls, SIG3_NoPuls, SIG4_NoPuls, SIG5_NoPuls, SIG6_NoPuls;

#define MINIMUMSP 100

#define WHEEL_PORT_12	PORTB

#define WHEEL1			PB4

#define WHEEL1_ON		WHEEL_PORT_12   |= (1<<WHEEL1)

#define WHEEL1_OFF		WHEEL_PORT_12   &=~ (1<<WHEEL1)


#define LEFT			2
#define RIGHT			1
#define NONE			0

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
extern void Timer1_Stop(void);

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
volatile unsigned short Speed1;
volatile unsigned short Speed2;
volatile unsigned short Speed3;
volatile unsigned short Speed4;
volatile unsigned short Speed5;
volatile unsigned short Speed6;
void Wild_Thumper_Init_All (void);

void PCI_Init (void);

#define WT_I2C_SLAVE_ADR 10
#define	extIntON	PORTC |=  (1<<PC2)
#define extIntOFF	PORTC &= ~(1<<PC2)

void PID_P(int Q);
void PID_I(int Q);
void PID_D(int Q);

#define PID_SCALING_FACTOR	32
#define TIME_TO_PID		44512UL //SIG_PID/SIG_VALUE TIME FOR 12 MAGNET POLES

#define DUTYCYCLE_PWM	256
#define WHEEL_PORT_12	PORTB
#define WHEEL_PORT_3456	PORTD

#define WHEEL1			PB4
#define WHEEL2			PB3
#define WHEEL3			PD7
#define WHEEL4			PD6
#define WHEEL5			PD5
#define WHEEL6			PD4

#define WHEEL1_ON		WHEEL_PORT_12   |= (1<<WHEEL1)
#define WHEEL2_ON		WHEEL_PORT_12   |= (1<<WHEEL2)
#define WHEEL3_ON		WHEEL_PORT_3456 |= (1<<WHEEL3)
#define WHEEL4_ON		WHEEL_PORT_3456 |= (1<<WHEEL4)
#define WHEEL5_ON		WHEEL_PORT_3456 |= (1<<WHEEL5)
#define WHEEL6_ON		WHEEL_PORT_3456 |= (1<<WHEEL6)

#define WHEEL1_OFF		WHEEL_PORT_12   &=~ (1<<WHEEL1)
#define WHEEL2_OFF		WHEEL_PORT_12   &=~ (1<<WHEEL2)
#define WHEEL3_OFF		WHEEL_PORT_3456 &=~ (1<<WHEEL3)
#define WHEEL4_OFF		WHEEL_PORT_3456 &=~ (1<<WHEEL4)
#define WHEEL5_OFF		WHEEL_PORT_3456 &=~ (1<<WHEEL5)
#define WHEEL6_OFF		WHEEL_PORT_3456 &=~ (1<<WHEEL6)

#define	FORWARD			2
#define BACKWARD 		1
#define BRAKE			0

#define LEFT			2
#define RIGHT			1
#define NONE			0
#define TURN			3


#define SIG1 	(PINC & (1<<PC6))
volatile bool SIG1_LastState;
volatile unsigned short SIG1_Counter;
volatile unsigned short SIG1_Value_up;
volatile unsigned short SIG1_Value_down;
volatile /*unsigned short*/ uint16_t SIG1_PID;

#define SIG2 	(PINC & (1<<PC5))
volatile bool SIG2_LastState;
volatile unsigned short SIG2_Counter;
volatile unsigned short SIG2_Value;
volatile /*unsigned short*/ uint16_t SIG2_PID;

#define SIG3 	(PINC & (1<<PC4))
volatile bool SIG3_LastState;
volatile unsigned short SIG3_Counter;
volatile unsigned short SIG3_Value;
volatile /*unsigned short*/ uint16_t SIG3_PID;

#define SIG4 	(PINC & (1<<PC3))
volatile bool SIG4_LastState;
volatile unsigned short SIG4_Counter;
volatile unsigned short SIG4_Value;
volatile /*unsigned short*/ uint16_t SIG4_PID;

#define SIG5 	(PIND & (1<<PD3))
volatile bool SIG5_LastState;
volatile unsigned short SIG5_Counter;
volatile unsigned short SIG5_Value;
volatile /*unsigned short*/ uint16_t SIG5_PID;

#define SIG6 	(PIND & (1<<PD2))
volatile bool SIG6_LastState;
volatile unsigned short SIG6_Counter;
volatile unsigned short SIG6_Value;
volatile /*unsigned short*/ uint16_t SIG6_PID;



volatile long int P_Factor, I_Factor, D_Factor, Error_MAX, SumError_MAX;
volatile long int SumError1,SumError2,SumError3,SumError4,SumError5,SumError6;
volatile long int LastProcessValue1, LastProcessValue2, LastProcessValue3, LastProcessValue4, LastProcessValue5, LastProcessValue6;
volatile unsigned int SetPoint1, SetPoint2, SetPoint3, SetPoint4, SetPoint5, SetPoint6; 

volatile long KP, KI, KD;

