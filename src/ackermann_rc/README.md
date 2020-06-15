# ackermann_rc ROS package

This package is an interface between an RC receiver connected to arduino and ROS. Arduino code is in Firmwares folder and instructions can be found for installation. Uses rosserial package for serial communication. System can be used for interfacing with racecar_simulator, morse simulations and 4/6 Wd platforms. Before using please tune the parameters of the receiver which can be found in param folder

## Dependencies

`sudo apt install ros-kinetic-rosserial-python`

## Launch files: Different control modes
 - `rc_driver_morse.launch` to drive the morse simulator cars
 - `rc_driver_vesc.launch` to drive the 4WD car directly sending speed commands to VESC OR using /cmd_val to interface our low-level controller(this has integration issues)
 - `rc_driver.launch` to publish AckermannDrive message to control racecar simulations of MIT
 - `rc_six_wheel.launch` to drive the 6WD car directly sending speed commands to 6WD motor drivers.

 To tune / modify the RC signals (button configs might change and need some tuning from time to time), use the *rc_params<mode>.yaml* under */param* folder.
