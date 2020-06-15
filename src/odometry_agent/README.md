# odometry_agent

In this package you can find the configurations about our odometry sources.

In `odometry_agent.launch` you can choose which sensors do you want to use or fuse with corresponding weights.

In `four/six_wheel_transforms.launch` you can find transforms for the sensors. Note that they are not placed precisely on the car.

`hector_configuration.launch`  sets parameters about hector_slam and starts slam algorithms.

Note that the most critical part of the package is Extended Kalman Filter (EKF), which publishes odom to base_link transform. EKF is started under ekf_template.launch and the related parameters and detailed explanations can be found under the `param/ekf_template.yaml file`

Note that in six wheeled car, the wheel odometry is not that reliable. So, we need to decrease its weight when fusing. On the other hand, four wheeled car's wheel odometry is better than its laser scan odometry. To change the weights we have two options. First of all, if you wish you can eliminate their use by just deleting their names from `param/ekf_template.yaml` file. The second way is more convenient but harder to implement. You can basically change covariance values at the packages (see the covariance variables of odometry msg types for both twist and pose messages). Note that smaller covariance means more precision. And on Rviz the position of your robot should always be in the covariance ellipse. The covariances of the odometry sensors of the cars are set under the files:
- vesc/vesc_ackermann/src/vesc_to_odom.cpp
- six_wheel_odom/src/six_wheel_odom.cpp
- rf2o_laser_odometry/src/CLaserOdometry2DNode.cpp
