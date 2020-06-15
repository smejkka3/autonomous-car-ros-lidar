#! /usr/bin/env morseexec

""" Basic MORSE simulation scene for <test> environment

Feel free to edit this template as you like!
"""

from morse.builder import *
from morse.sensors import *
from fourwd.builder.robots import Hummerscaled
from fourwd.builder.sensors.CustomBattery import Custombattery
import math

robot = Hummerscaled()
robot.add_default_interface('ros')
scale = 0.2
robot.properties(scale=scale)
robot.properties(GroundRobot=True)
robot.name = "FourWD"
robot.scale = [scale, scale, scale]

# This is the wheel odometry tf
odom = Odometry()
odom.add_stream('ros', frame_id="odom", topic="wheel_odom", child_frame_id='wheel_odom')
odom.alter('Noise', pos_std = 0.03, rot_std = math.radians(1.0), _2D = True)
odom.translate(0.0, 0.0, 0.0)
odom.rotate(0.0, 0.0, 0)

wheel_odom = Velocity()
wheel_odom.add_stream('ros', frame_id="wheel_odom", topic="wheel_odom_1")
#wheel_odom.alter('Noise', pos_std = 0.08, rot_std = math.radians(2), _2D = True)
wheel_odom.translate(0.0, 0.0, 0.0)
wheel_odom.rotate(0.0, 0.0, 0)


# IMU sensor located inside the camera
imu = IMU()
imu.name = "imu"
imu.add_stream('ros', frame_id='imu', topic='imu/data')
#imu.alter('Noise', pos_std = 0.025, rot_std = math.radians(1))
imu.translate(0.6, 0.0, 1.2)
imu.rotate(0.0, -math.pi/2, 0.0)

# Add a pose sensor that exports the current location and orientation. Note that this is only for testing purposes
#pose = Pose()
#pose.add_stream('ros', frame_id="map", topic='pose')

# Laser scanner for 360 degree
laser_scanner = Hokuyo()
laser_scanner.name = "laser_scan"
laser_scanner.add_stream('ros', frame_id="laser", topic="scan")
#laser_scanner.alter('Noise', pos_std = 0.05)
laser_scanner.translate(0.0, 0.0, 1.6)
laser_scanner.properties(resolution=1.0) #0.5 before
laser_scanner.properties(laser_range=25.0)
laser_scanner.properties(scan_window=360)
laser_scanner.properties(Visible_arc=False)
laser_scanner.rotate(0.0, 0.0, 0.0)
laser_scanner.create_laser_arc()

# The list of the main methods to manipulate your components
# is here: http://www.openrobots.org/morse/doc/astable/user/builder_overview.html
robot.translate(0.0, 0.0, 0.1)
robot.rotate(0.0, 0.0, 0.0)
robot.set_mass(1.5)

# Add a motion controller
# Check here the other available actuators:
# http://www.openrobots.org/morse/doc/stable/components_library.html#actuators
#
# 'morse add actuator <name> test' can help you with the creation of a custom
# actuator.
steerforce = SteerForce()
#steerforce.add_stream('ros', 'fourwd.middleware.ros.ackermann_ros.AckermannROS', topic='morse_steerforce_cmd')
steerforce.add_stream('ros', 'fourwd.middleware.ros.ackermann_ros.AckermannROS', topic='cmd_vel')

# place your component at the correct location
steerforce.translate(0, 0, 0)
steerforce.rotate(0, 0, 0)

robot.append(imu)
robot.append(laser_scanner)
robot.append(steerforce)
robot.append(odom)
robot.append(wheel_odom)

# a basic keyboard controller for testing purposes
keyboard = Keyboard()
robot.append(keyboard)

# set 'fastmode' to True to switch to wireframe mode
env = Environment('fourwd/environments/scaled_track_noisy.blend',fastmode = True) #tracks: scaled_track_noisy, scaled_track_simple complex_track.blend, simple_track.blend
env.set_camera_location([-18.0, -6.7, 10.8])
env.set_camera_rotation([1.09, 0, -1.14])
env.properties(latitude=1.53, longitude=45.1, altitude=0.0)
env.set_viewport(viewport_shade='TEXTURED', clip_end=1000)
env.show_framerate(True)
env.add_stream('ros')
