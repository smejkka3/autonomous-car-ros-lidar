//////////////////////////////////////////////////////////////////////////////////////
//                           _______________________
//                      \| WILD THUMPER  ROBOT SYSTEM |/
//                            \_-_-_-_-_-_-_-_-_-_/         >>> MOTOR CONTROLLER
// ----------------------------------------------------------------------------
// ------------------- [c]2010 / 2011 - AREXX ENGINEERING ---------------------
// -------------------------- http://www.arexx.com/ ---------------------------
//////////////////////////////////////////////////////////////////////////////////////
// File: 		Wild_Thumper_Motor.c
// Version: 	1.0
// Target: 		Wild Thumper Main Controller - ATMEGA664 @20.00 MHz
// Author(s): 	Bastiaan Scherphof 
// 			  	Hein Wielink 
//////////////////////////////////////////////////////////////////////////////////////
// Description:
// Control of the Wild Thumper
// This program handles:
// - I2C as master 
// - UART 0 
// - UART 1
//////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	includes Libraies																//
//																					//
//////////////////////////////////////////////////////////////////////////////////////
#include <avr/io.h>         		// General I/O Declarations
#include <avr/pgmspace.h>   		// label in flash
#include <avr/interrupt.h>  		// ext,timer int sei(), cli()
#include <inttypes.h>
#include <avr/eeprom.h>
#include <compat/deprecated.h>  	// stay compatible outp, sbi, cbi ed.
#include <stdbool.h> 
#include <stdio.h>
#include <string.h>
#include <compat/twi.h>
#include "WildThumperLib_Motor.h"	//library
#include "WT_UART.h"				//Include usart functions 
#include "WT_I2Cslave.h"			//Include I2C functions 


//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	I2C	(read registers)															//
//																					//
//////////////////////////////////////////////////////////////////////////////////////

uint8_t Test_Value = 0;

union {
 	uint8_t byte;
	struct {
		uint8_t powerOn:1;
		uint8_t ACSactive:1;
		uint8_t watchDogTimer:1;
		uint8_t wdtRequest:1;
		uint8_t wdtRequestEnable:1;
		uint8_t unused:3;
	};
} status;


//I2C Registers Read
#define I2C_REG_STATUS	 		 	1		//Status of the motor controller

#define I2C_REG_SPEED_MOTOR_1		2		//Actual speed of motor 1
#define I2C_REG_SPEED_MOTOR_2	 	3		//Actual speed of motor 2
#define I2C_REG_SPEED_MOTOR_3 	 	4		//Actual speed of motor 3
#define I2C_REG_SPEED_MOTOR_4	 	5		//Actual speed of motor 4
#define I2C_REG_SPEED_MOTOR_5	 	6		//Actual speed of motor 5
#define I2C_REG_SPEED_MOTOR_6 	 	7		//Actual speed of motor 6
#define I2C_REG_SPEED_ALL			8		//Actual speed of all motors

#define I2C_REG_CURR_MOTOR_1		10		//Actual current of motor 1
#define I2C_REG_CURR_MOTOR_2 	 	11		//Actual current of motor 2
#define I2C_REG_CURR_MOTOR_3 	 	12		//Actual current of motor 3
#define I2C_REG_CURR_MOTOR_4	 	13		//Actual current of motor 4
#define I2C_REG_CURR_MOTOR_5 	 	14		//Actual current of motor 5
#define I2C_REG_CURR_MOTOR_6 	 	15		//Actual current of motor 6

#define I2C_REG_ENCODER_ERRORS	 	16		//Status from the encoders
#define I2C_REG_MOTOR_ERRORS	 	17		//All error flags from the motors
#define I2C_REG_CURRENT_ERRORS	 	18		//All error flags from the motors

#define I2C_REG_LEDS	 		 	22		//Actual status of the four leds
#define I2C_TEST_I2C				23		//This register is used for test the I2C communication


// This function update all read registers.
void task_updateRegisters(void)
{
	if(!I2CTWI_readBusy) 
	{
		I2CTWI_readRegisters[I2C_REG_STATUS] = 			 	(uint8_t)(0);

		I2CTWI_readRegisters[I2C_REG_SPEED_MOTOR_1] = 	 	(uint8_t) SIG1_PID;
		I2CTWI_readRegisters[I2C_REG_SPEED_MOTOR_2] = 	 	(uint8_t) SIG2_PID;
		I2CTWI_readRegisters[I2C_REG_SPEED_MOTOR_3] = 	 	(uint8_t) SIG3_PID;
		I2CTWI_readRegisters[I2C_REG_SPEED_MOTOR_4] =   	(uint8_t) SIG4_PID;
		I2CTWI_readRegisters[I2C_REG_SPEED_MOTOR_5] = 	 	(uint8_t) SIG5_PID;
		I2CTWI_readRegisters[I2C_REG_SPEED_MOTOR_6] = 	 	(uint8_t) SIG6_PID;

		I2CTWI_readRegisters[I2C_REG_SPEED_ALL] = 	 		(uint8_t) Speed_Calculator();

		I2CTWI_readRegisters[I2C_REG_CURR_MOTOR_1] = 	 	(uint8_t)(GetADCValue_MotorCurrent(5));
		I2CTWI_readRegisters[I2C_REG_CURR_MOTOR_2] = 	 	(uint8_t)(GetADCValue_MotorCurrent(4));
		I2CTWI_readRegisters[I2C_REG_CURR_MOTOR_3] = 	 	(uint8_t)(GetADCValue_MotorCurrent(3));
		I2CTWI_readRegisters[I2C_REG_CURR_MOTOR_4] = 	 	(uint8_t)(GetADCValue_MotorCurrent(2));
		I2CTWI_readRegisters[I2C_REG_CURR_MOTOR_5] = 	 	(uint8_t)(GetADCValue_MotorCurrent(1));
		I2CTWI_readRegisters[I2C_REG_CURR_MOTOR_6] = 	 	(uint8_t)(GetADCValue_MotorCurrent(0));

		I2CTWI_readRegisters[I2C_REG_ENCODER_ERRORS] = 		(uint8_t) Check_Encoders();
		I2CTWI_readRegisters[I2C_REG_MOTOR_ERRORS] 	 = 		(uint8_t) Check_Motor_Flags();
		I2CTWI_readRegisters[I2C_REG_CURRENT_ERRORS] = 		(uint8_t) Check_Current();

		I2CTWI_readRegisters[I2C_REG_LEDS] = 			 	(uint8_t)(Shiftregister_2.byte);
		I2CTWI_readRegisters[I2C_TEST_I2C] = 			 	(uint8_t)(Test_Value);

		if(I2CTWI_dataWasRead && I2CTWI_dataReadFromReg == 0)  
		  {
		 	
		  }
	
	}
}

//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	I2C (Command registers)															//
//																					//
//////////////////////////////////////////////////////////////////////////////////////

//Variables for receive commands from the master 

uint8_t cmd;
uint8_t param1;
uint8_t param2;
uint8_t param3;
uint8_t param4;
uint8_t param5;
uint8_t param6;

#define I2C_REGW_CMD 0
#define I2C_REGW_CMD_PARAM1 1
#define I2C_REGW_CMD_PARAM2 2
#define I2C_REGW_CMD_PARAM3 3
#define I2C_REGW_CMD_PARAM4 4
#define I2C_REGW_CMD_PARAM5 5
#define I2C_REGW_CMD_PARAM6 6


// All Command registers	
#define CMD_STOP_ALL				1		//Command Wild Thumper: STOP WILD THUMPER
#define CMD_MOTORS_FORWARD_LEFT		2		//Command Wild Thumper: MOVE FORWARD / LEFT
#define CMD_MOTORS_FORWARD_RIGHT	3		//Command Wild Thumper: MOVE FORWARD / RIGHT
#define CMD_MOTORS_BACKWARD_LEFT	4		//Command Wild Thumper: MOVE BACKWARD / LEFT
#define CMD_MOTORS_BACKWARD_RIGHT	5		//Command Wild Thuper: MOVE BACKWARD / RIGHT 

#define CMD_CHANGE_MOTOR_1			10		//Command Wild Thuper: Speed motor 1
#define CMD_CHANGE_MOTOR_2			11		//Command Wild Thuper: Speed motor 2
#define CMD_CHANGE_MOTOR_3			12		//Command Wild Thuper: Speed motor 3
#define CMD_CHANGE_MOTOR_4			13		//Command Wild Thuper: Speed motor 4
#define CMD_CHANGE_MOTOR_5			14		//Command Wild Thuper: Speed motor 5
#define CMD_CHANGE_MOTOR_6			15		//Command Wild Thuper: Speed motor 6

#define CMD_DIFF_DRIVE				52		//Command Wild Thuper: Left Speed

#define CMD_SET_LEDS 				28		//Command Wild Thuper: Leds	
#define CMD_TEST_I2C				30		//Command Wild Thuper: I2C test

#define CMD_PID_P					33		//Command Wild Thuper: turn P on/off
#define CMD_PID_I					34		//Command Wild Thuper: turn I on/off
#define CMD_PID_D					35		//Command Wild Thuper: turn D on/off


// Checks if a new Command has been received and also reads all 
// paramters associated with this command.
// It returns true if a new command has been received.

uint8_t getCommand(void)
{
	if(I2CTWI_writeRegisters[I2C_REGW_CMD] && !I2CTWI_writeBusy) 
	{	
	
		cmd = I2CTWI_writeRegisters[I2C_REGW_CMD]; // store command register
		I2CTWI_writeRegisters[I2C_REGW_CMD] = 0; // clear command register (!!!)
		param1 = I2CTWI_writeRegisters[I2C_REGW_CMD_PARAM1]; // parameters 1-6...
		param2 = I2CTWI_writeRegisters[I2C_REGW_CMD_PARAM2];
		param3 = I2CTWI_writeRegisters[I2C_REGW_CMD_PARAM3];
		param4 = I2CTWI_writeRegisters[I2C_REGW_CMD_PARAM4];
		param5 = I2CTWI_writeRegisters[I2C_REGW_CMD_PARAM5];
		param6 = I2CTWI_writeRegisters[I2C_REGW_CMD_PARAM6];
		return true;
		
	}
	return false;
}


// Check the command and run the desired function
void task_commandProcessor(void)
{	

	if(getCommand()) //When received command
	{
		switch(cmd)  //Check which command is received
		{

			case CMD_STOP_ALL: 				Drive(BRAKE, 0, NONE,0);		break;
			case CMD_MOTORS_FORWARD_LEFT: 	Drive(FORWARD, param1, LEFT,param2);		break;   // "FOLLOW" // 
			case CMD_MOTORS_FORWARD_RIGHT:	Drive(FORWARD, param1, RIGHT,param2);		break;
			case CMD_MOTORS_BACKWARD_LEFT: 	Drive(BACKWARD, param1, LEFT,param2);		break;
			case CMD_MOTORS_BACKWARD_RIGHT: Drive(BACKWARD, param1, RIGHT,param2);		break;
			 
			case CMD_CHANGE_MOTOR_1: 	DriveWheel(1, param2, param1);break;
			case CMD_CHANGE_MOTOR_2: 	DriveWheel(2, param2, param1);break;
			case CMD_CHANGE_MOTOR_3: 	DriveWheel(3, param2, param1);break;
			case CMD_CHANGE_MOTOR_4: 	DriveWheel(4, param2, param1);break;
			case CMD_CHANGE_MOTOR_5: 	DriveWheel(5, param2, param1);break;
			case CMD_CHANGE_MOTOR_6: 	DriveWheel(6, param2, param1);break;
			
			case 52 :
			case 53 : 
			case 54 :
			case 55 : 	
			DriveDifferential(cmd, param2, param1);
			break;

			case CMD_SET_LEDS:			Shiftregister_2.byte = (Shiftregister_2.byte&0x0F) + param1; updateShiftRegisters(); break;
			case CMD_TEST_I2C:			Test_Value=param1+1;break;

			case CMD_PID_P:				PID_P(param2);break;
			case CMD_PID_I:				PID_I(param2);break;
			case CMD_PID_D:				PID_D(param2);break;
		}

	}
}


void task_MasterTimeout(void)
  {
	if(status.watchDogTimer)
	{
		if( getStopwatch2() > 3000)  // 3 seconds timeout for the master to react on
		{							 // our interrupt events - if he does not react, we 
									 // stop all operations!
			cli(); 					 // clear global interrupt bit!
			while(true) 			 // Rest In Peace
			{
			}
		}
		else if(getStopwatch3() > 250)
		{
			setStopwatch3(0);
		}
	}
  }


//main loop


int main (void)
{
	//////////////////////////////
	//   Configuration         //
	/////////////////////////////


	portInit();									// Configuration of Inputs and Outputs
		
	// USART0_Init(9600);							// Init USART0 

	Timer1_Init();								// Init Timer1

	ADC_Init();									// ADC init
	
	PWM_Init();

	PCI_Init();

	I2CTWI_initSlave(WT_I2C_SLAVE_ADR|TWI_GENERAL_CALL_ENABLE);			// Init I2C as Slave 

	sei();										// Enable interrupt	

	/////////////////////////////
	//   Start_Up functions    //
	/////////////////////////////

	Timer1_Start();								//Start Timer 1 (Enable Stopwatches and delay functions)


	StartUp_Ledblinking();						//By start-up led blinking 

	startStopwatch1();

	/////////////////////////////
	//   Main Loop             //
	/////////////////////////////
	
	KP = 1.7 * PID_SCALING_FACTOR; //setup P param for PID

	KI = 0.03 * PID_SCALING_FACTOR; //* PID_SCALING_FACTOR * 0; //setup I param for PID

	KD = 0.5 * PID_SCALING_FACTOR; //* PID_SCALING_FACTOR*0; //setup D param for PID

	PID_Init(KP, KI, KD); //Init PID values

	while(1==1)			
	{
		mSleep(4);
		Speed1 = PID_Controller(1,SetPoint1, SIG1_PID);
		Speed2 = PID_Controller(2,SetPoint2, SIG2_PID);
		Speed3 = PID_Controller(3,SetPoint3, SIG3_PID);
		Speed4 = PID_Controller(4,SetPoint4, SIG4_PID);
		Speed5 = PID_Controller(5,SetPoint5, SIG5_PID);
		Speed6 = PID_Controller(6,SetPoint6, SIG6_PID);
		task_commandProcessor();			
		task_updateRegisters();
	}
  }


