# VESC

VESC meta pakage contains many usefull codes for us. 

First of all  vesc_drives is the node that we use to interface VESC. It publishes many information about the state of the car with vesc_msgs/VescState file type



We also use vesc_ackermann/vesc_to_odom node which listens the node that ı previousely mentioned and publishes nav_msgs/odometry.

