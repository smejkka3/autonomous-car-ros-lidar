//////////////////////////////////////////////////////////////////////////////////////
//                           _______________________
//                      \| WILD THUMPER  ROBOT SYSTEM |/
//                            \_-_-_-_-_-_-_-_-_-_/         >>> MAIN CONTROLLER
// ----------------------------------------------------------------------------
// ------------------- [c]2010 / 2011 - AREXX ENGINEERING ---------------------
// -------------------------- http://www.arexx.com/ ---------------------------
//////////////////////////////////////////////////////////////////////////////////////
// File: 		Wild_Thumper_Main.c
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
// - APC Dongle communication
// - 
//////////////////////////////////////////////////////////////////////////////////////

//Include library
#include "WildThumperLib_Main.h"		//Include Wild Thumper functions
#include "WT_I2Cmaster.h"				//Include I2C functions
 
//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	Defines																			//
//																					//
//////////////////////////////////////////////////////////////////////////////////////

#define I2C_WT_ADR 					10				// The default address of the Master Controller 
#define INT0_STATUS_CHECK 			0				
#define WRITE_COMMAND				0				// 		
					
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


//I2C Registers Write (commands)

#define CMD_STOP_ALL				1		//Command Wild Thumper: STOP WILD THUMPER
#define CMD_MOTORS_FORWARD_LEFT		2		//Command Wild Thumper: MOVE FORWARD / LEFT
#define CMD_MOTORS_FORWARD_RIGHT	3		//Command Wild Thumper: MOVE FORWARD / RIGHT
#define CMD_MOTORS_BACKWARD_LEFT	4		//Command Wild Thumper: MOVE BACKWARD / LEFT
#define CMD_MOTORS_BACKWARD_RIGHT	5		//Command Wild Thumper: MOVE BACKWARD / RIGHT 

#define CMD_CHANGE_MOTOR_1			10		//Command Wild Thumper: CHANGE SPEED AND DIRECTION MOTOR 1
#define CMD_CHANGE_MOTOR_2			11		//Command Wild Thumper: CHANGE SPEED AND DIRECTION MOTOR 2
#define CMD_CHANGE_MOTOR_3			12		//Command Wild Thumper: CHANGE SPEED AND DIRECTION MOTOR 3
#define CMD_CHANGE_MOTOR_4			13		//Command Wild Thumper: CHANGE SPEED AND DIRECTION MOTOR 4
#define CMD_CHANGE_MOTOR_5			14		//Command Wild Thumper: CHANGE SPEED AND DIRECTION MOTOR 5
#define CMD_CHANGE_MOTOR_6			15		//Command Wild Thumper: CHANGE SPEED AND DIRECTION MOTOR 6
#define CMD_SPEED_MOTOR_1			16		//Command Wild Thumper: CHANGE SPEED MOTOR 1
#define CMD_SPEED_MOTOR_2			17		//Command Wild Thumper: CHANGE SPEED MOTOR 2
#define CMD_SPEED_MOTOR_3			18		//Command Wild Thumper: CHANGE SPEED MOTOR 3
#define CMD_SPEED_MOTOR_4			19		//Command Wild Thumper: CHANGE SPEED MOTOR 4
#define CMD_SPEED_MOTOR_5			20		//Command Wild Thumper: CHANGE SPEED MOTOR 5
#define CMD_SPEED_MOTOR_6			21		//Command Wild Thumper: CHANGE SPEED MOTOR 6
#define CMD_DIR_MOTOR_1				22		//Command Wild Thumper: CHANGE DIRECTION MOTOR 1
#define CMD_DIR_MOTOR_2				23		//Command Wild Thumper: CHANGE DIRECTION MOTOR 2
#define CMD_DIR_MOTOR_3				24		//Command Wild Thumper: CHANGE DIRECTION MOTOR 3
#define CMD_DIR_MOTOR_4				25		//Command Wild Thumper: CHANGE DIRECTION MOTOR 4
#define CMD_DIR_MOTOR_5				26		//Command Wild Thumper: CHANGE DIRECTION MOTOR 5
#define CMD_DIR_MOTOR_6				27		//Command Wild Thumper: CHANGE DIRECTION MOTOR 6
#define CMD_SET_LEDS 				28		//Command Wild Thumper: CHANGE LEDS
#define CMD_TEST_I2C				30		//This register is used for test the I2C communication 

#define CMD_PID_P					33
#define CMD_PID_I					34
#define CMD_PID_D					35

//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	Variables																		//
//																					//
//////////////////////////////////////////////////////////////////////////////////////

uint8_t block = false;				//Check if I2C line is free
uint8_t result[40];					//Received data from motor controller
uint8_t messageBuf[20]; 			// Buffer for I2C Data
uint8_t NumberOfI2Cerrors = 0;

//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	Received RF Data Handler														//
//																					//
//////////////////////////////////////////////////////////////////////////////////////

void WriteDataHandler_Main (void)
  {
    USART1_Write(1);								//Start Byte: 	1
    USART1_Write(255);								//Addres PC:	255
    USART1_Write(18);								//Length:		18
    USART1_Write(254);								//Command:		100
    USART1_Write(result[I2C_REG_SPEED_MOTOR_1]);					//Data 0: result[I2C_REG_STATUS]
 	USART1_Write(result[I2C_REG_ENCODER_ERRORS]);
	USART1_Write(result[I2C_REG_MOTOR_ERRORS]);
	USART1_Write(result[I2C_REG_CURRENT_ERRORS]);
    USART1_Write(result[I2C_REG_SPEED_ALL]);			
    USART1_Write(result[I2C_REG_CURR_MOTOR_1]);
    USART1_Write(result[I2C_REG_CURR_MOTOR_2]);
    USART1_Write(result[I2C_REG_CURR_MOTOR_3]);
    USART1_Write(result[I2C_REG_CURR_MOTOR_4]);
    USART1_Write(result[I2C_REG_CURR_MOTOR_5]);
    USART1_Write(result[I2C_REG_CURR_MOTOR_6]);
	USART1_Write(GetADCValue_BatteryVoltage());
	USART1_Write(GetADCValue_Temprature());
    USART1_Write(4);	 

  }


void WriteDataHandler_Test (void)
  {
    USART1_Write(1);								//Start Byte: 	1
    USART1_Write(255);								//Addres PC:	255
    USART1_Write(20);								//Length:		20
    USART1_Write(255);								//Command:		100
    USART1_Write(result[I2C_REG_SPEED_MOTOR_1]);	//Data 0:		
    USART1_Write(result[I2C_REG_SPEED_MOTOR_2]);
    USART1_Write(result[I2C_REG_SPEED_MOTOR_3]);
    USART1_Write(result[I2C_REG_SPEED_MOTOR_4]);
    USART1_Write(result[I2C_REG_SPEED_MOTOR_5]);
    USART1_Write(result[I2C_REG_SPEED_MOTOR_6]);
    USART1_Write(result[I2C_REG_CURR_MOTOR_1]);
    USART1_Write(result[I2C_REG_CURR_MOTOR_2]);
    USART1_Write(result[I2C_REG_CURR_MOTOR_3]);
    USART1_Write(result[I2C_REG_CURR_MOTOR_4]);
    USART1_Write(result[I2C_REG_CURR_MOTOR_5]);
    USART1_Write(result[I2C_REG_CURR_MOTOR_6]);
    USART1_Write(Sensor.byte);
	//USART1_Write(GetADCValue_BatteryVoltage());
	//USART1_Write(GetADCValue_Temprature());
	USART1_Write(GetADCValue_LDR2());
	USART1_Write(GetADCValue_LDR1());
	USART1_Write(4);	 
  }

//This function test the I2C communication,
//After test the I2C communication it generates a test repport,
//and send this to the Application.
void I2C_Test (void)
  {
  	uint8_t ReceivedWrongBytes = 0;		//Set receivedwrongbytes to 0
	NumberOfI2Cerrors = 0; 				//Set numberofI2Cerrors to 0
	sei();								//Enable interrupt 

  	for(int Test_Value =0; Test_Value<10; Test_Value++)				//For loop: Send/Read 10 times
	  {
        I2CTWI_transmit3Bytes(I2C_WT_ADR,WRITE_COMMAND, CMD_TEST_I2C, Test_Value); 		//Send Test value to Test register
        mSleep(30); 													//Wait 6ms
		I2CTWI_transmitByte(I2C_WT_ADR,0);									//Command for read:
		I2CTWI_readRegisters(I2C_WT_ADR,I2C_TEST_I2C,result,1);						//Read Test Register
	
		if( !(result[0]==(Test_Value+1)))							//If received byte is not Test value + 1
	  	  {
		     ReceivedWrongBytes++;									//One received byte!!!
	      }
      }
	  
    USART1_Write(1);					//Start Byte:  		1
    USART1_Write(255);					//Addres PC:   		255 
    USART1_Write(7);					//Length: 			7 bytes 
    USART1_Write(200);					//Control Byte:		200 (I2C test repport)
	USART1_Write(ReceivedWrongBytes);	//Data byte 0: 		Number of wrong received bytes
    USART1_Write(NumberOfI2Cerrors);	//Byte byte 1: 		Number of transmission errors 
    USART1_Write(4);					//Stop Byte:		4 		

  }

void RF_Test (void)
  {
  	sei();		
	for(int Test_Value_RF =0; Test_Value_RF<10; Test_Value_RF++)		
	  {
		USART1_Write(1);					//Start Byte:  		1
    	USART1_Write(255);					//Addres PC:   		255 
    	USART1_Write(6);					//Length: 			7 bytes 
    	USART1_Write(201);					//Control Byte:		200 (I2C test repport)
		USART1_Write(10);		//Data byte 0: 	
    	USART1_Write(4);					//Stop Byte:		4 		
	  
	  	mSleep(110);
	  }
	  

  }
void Drive_Control (char Direction, char Speed, char Angle)
  {
  	  Autonomous = false; 
   	  switch (Direction)
       	 {
		  case 0: I2CTWI_transmit4Bytes(I2C_WT_ADR, WRITE_COMMAND, CMD_STOP_ALL, 			Speed, Angle); break;
	 	  case 1: I2CTWI_transmit4Bytes(I2C_WT_ADR, WRITE_COMMAND, CMD_MOTORS_FORWARD_LEFT, Speed, Angle); break;
		  case 2: I2CTWI_transmit4Bytes(I2C_WT_ADR, WRITE_COMMAND, CMD_MOTORS_FORWARD_RIGHT, Speed, Angle); break;
		  case 3: I2CTWI_transmit4Bytes(I2C_WT_ADR, WRITE_COMMAND, CMD_MOTORS_BACKWARD_LEFT, Speed, Angle); break;
		  case 4: I2CTWI_transmit4Bytes(I2C_WT_ADR, WRITE_COMMAND, CMD_MOTORS_BACKWARD_RIGHT, Speed, Angle); break;
		  case 6: Autonomous = true; break;
		}
  }


//When a valid packet received from the APC220, this function is called. 
//This function check the control byte, and handles the data
void RF_ReceivedDataHandler1 (unsigned char ControlByte, char *InformationBytes)
  {	

	setStopwatch4(0);
	if (block == false)
	{
	  block = true;
   
	  switch (ControlByte)
       {		

	  	  //Move controls: All motors
		  // InformationBytes[0] = Speed
		  // InformationBytes[1] = Angle
		  case 0: I2CTWI_transmit4Bytes(I2C_WT_ADR, WRITE_COMMAND, CMD_STOP_ALL, 			InformationBytes[0], InformationBytes[1]); break;
	 	  case 1: I2CTWI_transmit4Bytes(I2C_WT_ADR, WRITE_COMMAND, CMD_MOTORS_FORWARD_LEFT, InformationBytes[0], InformationBytes[1]); break;
		  case 2: I2CTWI_transmit4Bytes(I2C_WT_ADR, WRITE_COMMAND, CMD_MOTORS_FORWARD_RIGHT, InformationBytes[0], InformationBytes[1]); break;
		  case 3: I2CTWI_transmit4Bytes(I2C_WT_ADR, WRITE_COMMAND, CMD_MOTORS_BACKWARD_LEFT, InformationBytes[0], InformationBytes[1]); break;
		  case 4: I2CTWI_transmit4Bytes(I2C_WT_ADR, WRITE_COMMAND, CMD_MOTORS_BACKWARD_RIGHT, InformationBytes[0], InformationBytes[1]); break;
		 	
		  //Move controls: One motor 
		  // InformationBytes[0] = Speed
		  // InformationBytes[1] = Direction
		  case 10:	I2CTWI_transmit4Bytes(I2C_WT_ADR, WRITE_COMMAND, CMD_CHANGE_MOTOR_1, InformationBytes[0],InformationBytes[1]); break;
		  case 11:	I2CTWI_transmit4Bytes(I2C_WT_ADR, WRITE_COMMAND, CMD_CHANGE_MOTOR_2, InformationBytes[0],InformationBytes[1]); break;
		  case 12:	I2CTWI_transmit4Bytes(I2C_WT_ADR, WRITE_COMMAND, CMD_CHANGE_MOTOR_3, InformationBytes[0],InformationBytes[1]); break;
		  case 13:	I2CTWI_transmit4Bytes(I2C_WT_ADR, WRITE_COMMAND, CMD_CHANGE_MOTOR_4, InformationBytes[0],InformationBytes[1]); break;
		  case 14:	I2CTWI_transmit4Bytes(I2C_WT_ADR, WRITE_COMMAND, CMD_CHANGE_MOTOR_5, InformationBytes[0],InformationBytes[1]); break;
		  case 15:	I2CTWI_transmit4Bytes(I2C_WT_ADR, WRITE_COMMAND, CMD_CHANGE_MOTOR_6, InformationBytes[0],InformationBytes[1]); break;

	 	  //Led controls: 
		  case 100: statusLEDs.byte = InformationBytes[0]; updateStatusLEDs();break; 
		  case 101: I2CTWI_transmit3Bytes(I2C_WT_ADR,WRITE_COMMAND, CMD_SET_LEDS, InformationBytes[0]); break; 
	
		  //Test Controls
		  case 200:  I2C_Test(); break;
		  case 201:  RF_Test();  break;

		  //PID controls
		  case 240:  I2CTWI_transmit4Bytes(I2C_WT_ADR, WRITE_COMMAND, CMD_PID_P, InformationBytes[0],InformationBytes[1]); break; //P
		  case 241:  I2CTWI_transmit4Bytes(I2C_WT_ADR, WRITE_COMMAND, CMD_PID_I, InformationBytes[0],InformationBytes[1]); break; //I
		  case 242:  I2CTWI_transmit4Bytes(I2C_WT_ADR, WRITE_COMMAND, CMD_PID_D, InformationBytes[0],InformationBytes[1]); break; //D
	
		  //Data Controls
		  case 254: WriteDataHandler_Main(); 
		  			Drive_Control(InformationBytes[0], InformationBytes[1], InformationBytes[2] );
		  			break; 
		  
		  case 255: WriteDataHandler_Test(); break;	
	   }

	   block = false; 
	   setStopwatch5(0);
     }

	
  }


//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	I2C	functions																	//
//																					//
//////////////////////////////////////////////////////////////////////////////////////

//  I2C interrupt Handler 
void I2C_Event_Handler(void)
  {
    // this code will be called anytime that PCINT31 switches 
    //    (hi to lo, or lo to hi)

	if(!block && (PIND & (1<< PIND7))) 
	{
		block = true; // Block further requests and wait until 
					  // this request has been processed.
		I2CTWI_requestRegisterFromDevice(I2C_WT_ADR, INT0_STATUS_CHECK, 0, 20);

	}

  }


// I2C interrupt Handler 
// This Event Handler is very nice for reacting on an interrupt request 
// from the Slave controller and read all the data from it! 
void I2C_requestedDataReady(uint8_t dataRequestID)
  {
	if(dataRequestID == INT0_STATUS_CHECK) 
	  {                                      
	  	// get received data (6 bytes)
        I2CTWI_getReceivedData(result, 20); 
		
		
	   	//Action: 
		//................
		// statusLEDs.byte = messageBuf[0];
		// updateStatusLEDs();

		// ------------------------------------
		// IMPORTANT - reset the block flag:

		block = false;
	  }
  }

//  I2C error Handler
void I2C_transmissionError(uint8_t errorState)
  {
    NumberOfI2Cerrors++;
	USART0_WriteString("\nI2C ERROR - TWI STATE: 0x");
	USART0_Write(errorState);
	block = false;
  }



void task_Sensors (void)
  {
	Sensor.BUMPER_R = get_bumper_right();
	Sensor.BUMPER_L = get_bumper_left();
	Sensor.ACS_R = ACS_Check_Right();
	mSleep(1);
	Sensor.ACS_L = ACS_Check_Left();
	mSleep(1);
	Sensor.ACS_RF = ACS_Check_Front_Right();
	mSleep(1);
	Sensor.ACS_LF = ACS_Check_Front_Left();	

  }

void task_Application_Timeout (void)
  {
	if(getStopwatch5() > 1000)
	  	  {
			I2CTWI_transmit4Bytes(I2C_WT_ADR, WRITE_COMMAND, CMD_STOP_ALL, 0, 0);
		  }

  }

void Drive_autonomous(void)
  { 

   	if (getStopwatch6() > 2000)
  	  {
		I2CTWI_transmit4Bytes(I2C_WT_ADR, WRITE_COMMAND, CMD_MOTORS_FORWARD_LEFT, 80, 0); 
		setStopwatch6(0);
		stopStopwatch6();
		Obstacle_Right = false; 
		Obstacle_Left = false; 
	  }

	else if (Sensor.BUMPER_R || Sensor.ACS_R || Sensor.ACS_RF || Obstacle_Right)
	  {
	  	startStopwatch6();
 		if (Obstacle_Right )I2CTWI_transmit4Bytes(I2C_WT_ADR, WRITE_COMMAND, CMD_MOTORS_BACKWARD_RIGHT, 70, 6); 
		Obstacle_Right = true; 
	  }

	 else if (Sensor.BUMPER_L || Sensor.ACS_L || Sensor.ACS_LF || Obstacle_Left)
      {
	 	startStopwatch6();
		if (Obstacle_Left)	I2CTWI_transmit4Bytes(I2C_WT_ADR, WRITE_COMMAND, CMD_MOTORS_BACKWARD_LEFT, 70, 6); 
		Obstacle_Left = true; 
	  }
	
	 else I2CTWI_transmit4Bytes(I2C_WT_ADR, WRITE_COMMAND, CMD_MOTORS_FORWARD_LEFT, 80, 0); 

  }

//////////////////////////////////////////////////////////////////////////////////////
//																					//
//  Main loop																		//
//																					//
//////////////////////////////////////////////////////////////////////////////////////

int main (void)
  {
	//////////////////////////////
	//   Configuration          //
	//////////////////////////////
	
	portInit();							//Set Input and Outputs

	USART0_Init(9600);					//Init USART0 

	USART1_Init(9600);					//Init USART1

	Timer1_Init();						//Init Timer1

	I2CTWI_initMaster(200);				//Init I2C as master (100kHz clock)
	
	ADC_Init();

	initACS();

	PCMSK3 |= (1 << PCINT31);			
  	PCICR |= (1 << PCIE3);
	EICRA = (1<<ISC21)|(0<<ISC20);
	EIMSK = (1<<INT2);	

	sei();								// Enable interrupt	

	
	/////////////////////////////
	//   Start_Up functions    //
	/////////////////////////////
	
	Timer1_Start();									//Start Timer 1 (Enable Stopwatches and delay functions)

	StartUp_Ledblinking();							//By start-up led blinking

	mSleep(500);

	LedOnOff(0b00101010);

	USART0_WriteString("Gestart Wild Thumper");		//Write to Uart

	sei();
	
	//////////////////////////////////////
	//   Register Event Handlers I2c    //
	//////////////////////////////////////

	I2CTWI_setTransmissionErrorHandler(I2C_transmissionError);

	I2CTWI_setRequestedDataReadyHandler(I2C_requestedDataReady);

	I2C_setInterruptEventHandler(I2C_Event_Handler);

	RF_SetReceivedDataHandler(RF_ReceivedDataHandler1);

	startStopwatch4();
	startStopwatch5();



	/////////////////////////////
	//   Main Loop             //
	/////////////////////////////

	while(1==1)			
	  {

	  	if(getStopwatch4() > 100 && block == false)
	  	  {
	    	//update of the register from the motor controller
			//This will happend every 200ms
			block = true; 
			I2CTWI_transmitByte(10,0);
			I2CTWI_readRegisters(10,0,result,20);
			block = false;
			setStopwatch4(0);
	  	  }
	
	  	task_I2CTWI();
	  	task_Sensors(); 
		task_Application_Timeout();
		
		if (Autonomous == true && (getStopwatch5() < 1000))

	 	  {
		  Drive_autonomous();
	  	  }
	  }
  }


