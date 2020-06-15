
# serial_6w package

This package, respectively node, is responsible for the communication between main MCU of the motor controller 
(Arexx WTR-CK1) of the 6WD car and PC.

Note that the default protocol for six wheeled car was not usable for our cases. For this reason we have created
two own protocols and adjusted the Firmware accordingly (see **[Firmware](../../Firmwares/SixWheelCar_Firmware/README.md)**).

The node of this package listens to the topic `motor_commands`, which is using the `SixWheelCommand.msg` message type and
publishes some status information on the topic `motor_controller_info` with the message type `SixWheelInfo.msg`.


At SixWheelCommand.msg if you set control type (`controltype`) to 0 you can set right and left side speed separately.
if you set `controltype` to 1 the car uses its own control method where it accepts only speed and angle as input. If it gets an angle it slows down the tires at the turning direction according to the formula: 

`Speed = Given_Speed-(Given_Speed*Angle/125)`  

if you set `controltype` to 2 you can control tires separately.

## PC to MCU communication

Between MCU and ROS communications we are only using bytes though.

Please refer to **[Firmware ReadMe](../../Firmwares/SixWheelCar_Firmware/README.md)** for details about the communication protcol.

This nodes expects the serial device to be available as `/dev/ttyUSB_thumper`, please check related udev configuration
in **[Linux Setup](../../linux_env/README.md)** for further information.