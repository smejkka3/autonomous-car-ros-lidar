#SixwheelFirmware

`/Custom` MCU is the being used
`/Original` was sent by the AREX company and it is not used.

## Serial Communication Protocols

### Communication Between MAIN_MCU and PC (UART - RS432):
The introduction and ending of the message is always same. And you can see an example byte array to control the car.

1. --> Start Byte (1)
2.	--> MCU Address (255)
3. 	--> Message Length
4.	--> Control Type 
5. 	--> Info Bytes
6.	--> Info Bytes
7. 	--> Info Bytes
8. 	--> Info Bytes
9.	--> End Byte (4)

***We used three control types, explained below***

##### 1) The one we control left and right separately

1. 	--> Start Byte(1)
2.	--> MCU Address(255)
3. 	--> Message Length (7)
4.	--> Direction Select (52,53,54 or 55)
5. 	--> Info Bytes (Right Speed)
6.	--> Info Bytes (Left Speed)
7.	--> End Byte (4)

To choose Control Type byte
* 52: Both right and left side goes forward
* 53: Right side backward left side forward
* 54: Right side forward left side backward
* 55: Both right and left side goes backward

##### 2) Default Controller 

It gets a speed and sets the all motors speeds. If it gets an angle it slows down the tires at the turning direction according to the formula: 

`Speed = Given_Speed-(Given_Speed*Angle/125)`  

1. 	--> Start Byte(1)
2.	--> MCU Address(255)
3. 	--> Message Length (8)
4.	--> Control Type (51)
5. 	--> Direction Select (1,2,3 or 4)
6.	--> Info Bytes (speed)
7.	--> Info Bytes (angle)
8.	--> End Byte (4)

* 1: Go Forward Turn clockwise
* 2: Go Forward Turn counter clockwise
* 3: Go Backward Turn clockwise
* 4: Go Backward Turn counter clockwise

##### 3) Individual motor control

1. 	--> Start Byte(1)
2.	--> MCU Address(255)
3. 	--> Message Length (7)
4.	--> Control Type (motor number+9)
5. 	--> Info Bytes (speed)
6.	--> Info Bytes (direction)(1 forward or 2 backward)
7.	--> End Byte (4)

### Communication Between MAIN MCU and MOTOR MCU (I2C):

The Protocol is actually really simple. The main MCU just forwards the received bytes to the motor MCU. Motor MCU processes them and creates required speeds for each motor.
