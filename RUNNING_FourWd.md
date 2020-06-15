# How to Run

## Four Wheeled (4WD) Platform

### Connecting to the Jetson PC

Connect to the *CARS* wifi with password *Test1234*
Then, ssh terminals to Jetson PC:  

`ssh jetson2@192.168.1.188`
password for the jetson2: *jetsongtarc*

### Running the project

#### Running Autonomous Navigation (Real Car)

System is compromised of four basic components.

- Motor Controller Interface which takes cmd_vel message and translates it into motor commands.
- Odometry agent which initializes sensors and starts slam algorithms. Also publishes necessary transform trees.
- Local planner which calculates cmd_vel given odometry and path data
- Global planner which calculates the path from start to the end.

You can run the entire autonomous navigation stack with

`roslaunch bear_car_launch autonomous_navigation_fourwd.launch`

For a way to give an autonomous driving goal, please refer to the Visualization title below.

#### Running Autonomous Navigation Step-by-Step (Real Car)

If you only want to follow the process manually, please follow commands below.

To run sensor nodes separately:

`roslaunch bear_car_launch sensors.launch`
However, sensors are automatically started by the odometry agent below.

To run the motor controller:

`roslaunch vesc_driver vesc_driver_node.launch`

**TODO: Write the agent names comes with the roslaunch (and their brief descriptions)**

This starts ROS node for communicating with servo and BLDC.  Node launched is the vesc_driver_node. This agent handles all serial communication to VESC. It sends commands and periodically polls sensor data from VESC.

Next step is to start the low level speed and steering controller. These nodes take cmd_vel and translates it into correct messages for vesc_driver_node.



To start linear controller together with PID controller node:

`roslaunch low_level_speed_controller allg.launch is_four_wd:=true`

If PID controller is started separately (e.g. if you use navigation_stack.launch) you have to use

`roslaunch low_level_speed_controller four_wd.launch`

**TODO: Write the agent names comes with the roslaunch (and their brief descriptions)**

This starts the controller node with parameters in param folder. Controller is responsible for taking cmd_vel and generating necessary commands for VESC driver. This node also handles takeoff behaviour.


To start the steering controller:

`rosrun low_level_steering_controller low_level_steering_controller`

This is only an interface which takes rad/s and translates it into servo commands. Details can be found in the  architecture part.


To start odometry and sensors together:

`roslaunch odometry_agent odometry_agent.launch is_four_wd:=true`


This package launches hector_slam, LIDAR driver rplidar_ros and realsense camera driver. All data from sensors are fused using robot_localization package (an EKF is launched for the operation). IMU data is retrieved from realsense camera. Currently laser odometry and wheel odometry are used. IMU was causing oscillations in the pose; therefore, its weight (effect) is really small in EKF sensor fusion. This may be adjusted under `/src/odometry_agent/param/ekf_templage.yaml`.

Additionally, some of the localization options are left optional to run under `odometry_agent.launch` file. Add the following parameters to the roslaunch command above to activate or deactive them (by default all are active): `wheel_odometry:=true`, `imu_odometry:=false`, `laser_scanner_odometry:=true`.


To start all planners and pid controllers for them:

`roslaunch pose_follower navigation_stack_fourwd.launch`

This will make the car run in autonomous mode.

#### Running 4WD Simulation

A simulation environment is prepared for the 4WD car in MORSE environment. Make sure that you have latest MORSE master installed.
To run, first we need to import the MORSE project, then simply run the builder script:

```bash
cd <project_folder>/morse
morse import fourwd  # replace sixwd for 6WD car
morse run fourwd fourwd/default.py  # replace sixwd with fourwd
```

This will initialize the environment. Note that you need relevant ROS nodes or projects running that interfaces the MORSE environment. If you would like to navigate in the environment, at least start a `roscore` so the interface between MORSE and ROS initializes.

In simulation, odometry_agent is tested and successfully integrated to run. The goal is to develop our own motion controller (this will be specific to the simulation as the simulated car models do not reflect the motor behaviors of the real cars), own path planner and own decision-making agents. The latter two are intended to work also on the real cars; therefore, the motion controller interface with the planners should be the same as in the real cars.

To run odometry agent on the simulation, this time we run with the parameter that is stating it is simulation

```bash
roslaunch odometry_agent odometry_agent.launch is_four_wd:=true is_simulation:=true
# is_four_wd:=false for 6Wd car simulation
```

If you launch rviz again as stated below, the localization will be visible.

To control the robot, you need to publish to the topic below. *Linear.x* value is for linear speed, *angular.z* for angular speed.

```bash
rostopic pub /cmd_vel geometry_msgs.Twist:
```
Optionally, you can control the car by also running teleop package. It allows you to command the car with keyboard:

```bash
roslaunch bear_car_launch morse_teleop.launch
```
If the terminal teleop is launched does not provide keyboard control interface, you can also rosrun the teleop package:
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

**TODO**: test the local planner nodes for the simulation

#### Visualization (Real Car and Simulation)

If you run in the Jetson PC directly (with a monitor), run rviz to control the car and its navigation:
`rosrun rviz rviz`

Also With the configuration:

```bash
rosrun rviz rviz -d `rospack find pose_follower`/cfg/rviz_navigation.rviz
```

This is also wrapped within

`roslaunch pose_follower navigation_stack_rviz.launch`

In order to give a goal point, you can use 2D nav goal interface on rviz UI. 2D_nav_goal, when clicked, will allow you to put a point on the map. When the point is selected with the mouse, the car should autonomously navigate to the location.

#### RC mode for running the project and to control the simulation

To start the **RC mode**, there are two options currently implemented.

First option is to use rc_driver_vesc launch file in ackermann_rc package. This launch file launches a node that directly interfaces with vesc_driver. Hence using this launch file with low_level_speed and steer controllers is not recommended.

Second option is to use the interface created for morse. This is included in rc_driver_morse launch file. Morse accepts twist message for control and that twist output can be fed into low level controllers, which would result in a better control.

Third, currently not fully implemented but possible solution is to use rc_driver launch file. This node outputs AckermannDrive msg and linear low speed controller can handle this message. But steering controller lacks a simple listener to this message type.


To run in RC mode, *DO NOT launch pose_follower navigation_stack.launch*. To run in RC mode with our low-level controller (integrated, but issues remain. See the open issues page), assign `true` to `use_twist` parameter under *ackermann_rc/param/rc_driver_vesc.yaml*, then run the rc_driver_vesc.launch below:

`roslaunch ackermann_rc rc_driver_vesc.launch`


To run using morse interface, RC interface is also created. However, for the ease of use, we strongly recommend developing a teleop packages to be able to control the car with the keyboard.

`roslaunch ackermann_rc rc_driver_morse.launch`

None of these launch files require odometry_agent to be running.

You can also directly run the entire setup with just one launch file for direct rc control with

`roslaunch bear_car_launch direct_rc_fourwd.launch`

### Tuning the control parameters

For the purpose, a YAML file is created to interface the low level control units. This directly loads the parameters to the VESC driver through VESC tool. Some details about the VESC driver in general, how to load the params and interface it is given under this wiki page:

`/wiki/vesc`

The interface in general covers:
- `low_level_speed_controller` pkg interfacing with VESC package (translation of speed commands to VESC motor control signals)
- VESC Tool configuration: please refer to `Firmware/FourWheelCarConfigurations(VESC)/README.md`
- Remote controller interface to VESC: param set under `ackermann_rc/param/rc_driver_vesc.yaml`
- PID control: PID parameters are set under /pose_follower package. This package decides the speed values to be commanded to the motor drivers (VESC for fourwd). The parameters can be changed from `pid_controller.launch` under *pose_follower* package.
- **TODO: how to give a goal point for navigation (writing to a rostopic/callign a service)**

**TODO: here write how to add the config file**

#### Remaining Motor Control Issues:
Some issues that needs to be resolved regarding fourwd car's VESC and motor drive:
- Extend VESC firmware to incorporate encoder speed sending. This is referred under wiki page: Wiki-->vesc
- low_level_speed_controller needs a fine tuning along with VESC driver parameters (to be set under VESC firmware and VESC tool) to satisfy a proper take-off and break behaviors.

### Emergency Stop

For emergency stop, there are couple of methods. Killing the terminal with **low_level_speed_controller** shuts down communication with VESC. This causes VESC to stop in a short while usually under a second.

If **navigation_stack** is killed, cmd_vel generation is stopped. This causes low_level_speed_controller to stop the car after command_timeout time is passed. This parameter is changeable from corresponding yaml file of low_level_speed_controller.

Currently, there is only one hardware kill switch on the battery that supplies digital boards. Closing it would shutdown motors immediately.


### Interfacing

#### VESC Driver:

​	/sensors/core: This topic outputs the internal values of VESC like erpm, input voltage, current, temp...

/commands/motor/... : There are several topics like this. They receive double values to corresponding interfaces. For example a 0.04 published to /commands/motor/duty_cycle causes a switch to duty_cycle control with 4% duty_cycle.

#### Low Level Controllers

​	/cmd_vel: This basically the only input topic they need. They also listen to the odometry data. Topic names can be changed from the yaml file four_wd_params.yaml. In that file you can also configure takeoff duty_cycle levels.

#### Local And Global Planners

They generate the cmd_vel necessary to complete the path. All PID controller parameters can be changed from`pose_follower/launch/pid_controller.launch`.  Also in cfg/carlike folder, all parameters necessary for the system is present. In pose_follower.yaml you can change goal point tolerance and goal point timeout as well as controller limits.  To send a goal, you can use rviz 2D nav goal interface. Also move_base also has an action interface to send goal points.


* TODO Services, topics their descriptions (high-level)
* TODO: global planner practical parameters (the ones that launch file imports). For all the rest provide the online links**
* TODO: local planner practical parameters, e.g. how to tune PID parameters, reaching goal point etc.**
* TODO: local cost map parameters (from move base)**
* TODO: how to send path arrays (overwriting the global planner)**
* TODO: odometry: Localization does fusion (how to change the weights of odometries?) and how to select the sensors/types/values from the launch file **
