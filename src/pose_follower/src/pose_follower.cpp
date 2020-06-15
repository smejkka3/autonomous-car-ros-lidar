/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <nav_msgs/Path.h>
#include <pose_follower/pose_follower.h>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(pose_follower::PoseFollower, nav_core::BaseLocalPlanner)


namespace pose_follower {
  PoseFollower::PoseFollower(): tf_(NULL), costmap_ros_(NULL) {}

  void PoseFollower::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros){
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    current_waypoint_ = 1;
    goal_reached_time_ = ros::Time::now();
    ros::NodeHandle node_private("~/" + name);

    collision_planner_.initialize(name, tf_, costmap_ros_);

    node_private.param("k_trans", K_trans_, 2.0);
    node_private.param("k_rot", K_rot_, 2.0);

    //within this distance to the goal, finally rotate to the goal heading (also, we've reached our goal only if we're within this dist)
    node_private.param("tolerance_trans", tolerance_trans_, 0.02); 

    //we've reached our goal only if we're within this angular distance
    node_private.param("tolerance_rot", tolerance_rot_, 0.04);

    //we've reached our goal only if we're within range for this long and stopped
    node_private.param("tolerance_timeout", tolerance_timeout_, 0.5);

    //set this to true if you're using a holonomic robot
    node_private.param("holonomic", holonomic_, true);

    //number of samples (scaling factors of our current desired twist) to check the validity of 
    node_private.param("samples", samples_, 10);

    //go no faster than this
    node_private.param("max_vel_lin", max_vel_lin_, 0.9);
    node_private.param("max_vel_th", max_vel_th_, 1.4);

    //minimum velocities to keep from getting stuck
    node_private.param("min_vel_lin", min_vel_lin_, 0.1);
    node_private.param("min_vel_th", min_vel_th_, 0.0);

    //if we're rotating in place, go at least this fast to avoid getting stuck
    node_private.param("min_in_place_vel_th", min_in_place_vel_th_, 0.0);

    //when we're near the end and would be trying to go no faster than this translationally, just rotate in place instead
    node_private.param("in_place_trans_vel", in_place_trans_vel_, 0.0);

    //we're "stopped" if we're going slower than these velocities
    node_private.param("trans_stopped_velocity", trans_stopped_velocity_, 1e-4);
    node_private.param("rot_stopped_velocity", rot_stopped_velocity_, 1e-4);

    //if this is true, we don't care whether we go backwards or forwards
    node_private.param("allow_backwards", allow_backwards_, false);

    //if this is true, turn in place to face the new goal instead of arcing toward it
    node_private.param("turn_in_place_first", turn_in_place_first_, false);

    //if turn_in_place_first is true, turn in place if our heading is more than this far from facing the goal location
    node_private.param("max_heading_diff_before_moving", max_heading_diff_before_moving_, 0.17);
    node_private.param("K_linear", K_linear, 0.8);
    node_private.param("K_rotational", K_rotational, 0.3);
    node_private.param("wheelbase", wheelbase, 0.325);
    node_private.param("max_turn_angle", max_turn_angle, 0.5);

    control_effort_topic = "/linear_pid/control_effort";
    setpoint_topic = "/linear_pid/setpoint";
    state_topic = "/linear_pid/state";
    enable_topic = "/linear_pid/pid_enable";

    steer_control_effort_topic = "/steer_pid/control_effort";
    steer_setpoint_topic = "/steer_pid/setpoint";
    steer_state_topic = "/steer_pid/state";
    steer_enable_topic = "/steer_pid/pid_enable";

    node_private.param("control_effort_topic", control_effort_topic, control_effort_topic);
    node_private.param("setpoint_topic", setpoint_topic, setpoint_topic);
    node_private.param("state_topic", state_topic, state_topic);
    node_private.param("enable_topic", enable_topic, enable_topic);

    node_private.param("steer_control_effort_topic", steer_control_effort_topic, steer_control_effort_topic);
    node_private.param("steer_setpoint_topic", steer_setpoint_topic, steer_setpoint_topic);
    node_private.param("steer_state_topic", steer_state_topic, steer_state_topic);
    node_private.param("steer_enable_topic", steer_enable_topic, steer_enable_topic);

    executing_goal = false;

    ROS_INFO("Holonomic: %d", holonomic_);

    global_plan_pub_ = node_private.advertise<nav_msgs::Path>("global_plan", 1);

    ros::NodeHandle node;
    odom_sub_ = node.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind(&PoseFollower::odomCallback, this, _1));
    vel_pub_ = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    control_effort_sub = node_private.subscribe(control_effort_topic, 100, &PoseFollower::control_effort_callback, this);
    state_pub = node_private.advertise<std_msgs::Float64>(state_topic, 10, false);
    setpoint_pub = node_private.advertise<std_msgs::Float64>(setpoint_topic, 10, false);
    enable_pub = node_private.advertise<std_msgs::Bool>(enable_topic, 10, false);


    steer_control_effort_sub = node_private.subscribe(steer_control_effort_topic, 100, 
                                &PoseFollower::steer_control_effort_callback, this);
    steer_state_pub = node_private.advertise<std_msgs::Float64>(steer_state_topic, 10, false);
    steer_setpoint_pub = node_private.advertise<std_msgs::Float64>(steer_setpoint_topic, 10, false);
    steer_enable_pub = node_private.advertise<std_msgs::Bool>(steer_enable_topic, 10, false);

    std_msgs::Bool enable_msg;
    enable_msg.data = true;
    enable_pub.publish(enable_msg);
    steer_enable_pub.publish(enable_msg);

    std_msgs::Float64 setpoint_msg;
    setpoint_msg.data = 0.0;
    setpoint_pub.publish(setpoint_msg);
    steer_setpoint_pub.publish(setpoint_msg);

    linear_control_effort = 0.0;
    steer_control_effort = 0.0;
    ROS_INFO("Initialized");
  }

  void PoseFollower::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    //we assume that the odometry is published in the frame of the base
    boost::mutex::scoped_lock lock(odom_lock_);
    base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
    base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
    base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
    std_msgs::Bool enable_msg;
    
    //if(executing_goal){
      enable_msg.data = true;
      std_msgs::Float64 state;
      state.data = distBetweenPoints(msg->pose.pose, current_closest_point.pose);
      state_pub.publish(state);
      
      enable_pub.publish(enable_msg);
      steer_enable_pub.publish(enable_msg);

   // }//else{
     // enable_msg.data = false;
     // enable_pub.publish(enable_msg);
     // steer_enable_pub.publish(enable_msg);
   // }
    

    //ROS_INFO("In the odometry callback with velocity values: (%.2f, %.2f, %.2f)",
        //base_odom_.twist.twist.linear.x, base_odom_.twist.twist.linear.y, base_odom_.twist.twist.angular.z);
  }

  double PoseFollower::headingDiff(double x, double y, double pt_x, double pt_y, double heading)
  {
    double v1_x = x - pt_x;
    double v1_y = y - pt_y;
    double v2_x = cos(heading);
    double v2_y = sin(heading);

    double perp_dot = v1_x * v2_y - v1_y * v2_x;
    double dot = v1_x * v2_x + v1_y * v2_y;

    //get the signed angle
    double vector_angle = atan2(perp_dot, dot);

    return -1.0 * vector_angle;
  }

  bool PoseFollower::stopped(){
    //copy over the odometry information
    nav_msgs::Odometry base_odom;
    {
      boost::mutex::scoped_lock lock(odom_lock_);
      base_odom = base_odom_;
    }

    return fabs(base_odom.twist.twist.angular.z) <= rot_stopped_velocity_
      && fabs(base_odom.twist.twist.linear.x) <= trans_stopped_velocity_
      && fabs(base_odom.twist.twist.linear.y) <= trans_stopped_velocity_;
  }

  void PoseFollower::publishPlan(const std::vector<geometry_msgs::PoseStamped> &path,
                               const ros::Publisher &pub) {
    // given an empty path we won't do anything
    if (path.empty())
      return;

    // create a path message
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());
    gui_path.header.frame_id = path[0].header.frame_id;
    gui_path.header.stamp = path[0].header.stamp;

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
      gui_path.poses[i] = path[i];
    }
    pub.publish(gui_path);
  }

  double PoseFollower::distBetweenPoints(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b){
    return std::sqrt(std::pow(a.position.x - b.position.x,2) + std::pow(a.position.y - b.position.y,2));
  }

  double PoseFollower::curvature(const geometry_msgs::PoseStamped & start,
                   const geometry_msgs::PoseStamped & end ) {
    double dx = (end.pose.position.x - start.pose.position.x);
    double dy = (end.pose.position.y - start.pose.position.y);
    double yaw1 = tf2::getYaw(start.pose.orientation);
    double yaw2 = tf2::getYaw(end.pose.orientation);
    double dtheta = fabs(angles::shortest_angular_distance(yaw1, yaw2));
    double ds = hypot(dx, dy);
    return dtheta / ds;
  }

  double PoseFollower::slope(const geometry_msgs::PoseStamped & start,
                   const geometry_msgs::PoseStamped & end ) {
      
    double dx = (end.pose.position.x - start.pose.position.x);
    double dy = (end.pose.position.y - start.pose.position.y);

    return dy / dx;
  }

  int PoseFollower::getClosestPoint(std::vector<geometry_msgs::PoseStamped> &global_plan, geometry_msgs::Pose pose){
    int best = 0;
    double dist = 10000.0;

    for(int i = 0; i < global_plan.size(); i++){
      if(distBetweenPoints(global_plan[i].pose, pose) < dist){
        best = i;
        dist = distBetweenPoints(global_plan[i].pose, pose);
      }
    }

    return best;
  }

  bool PoseFollower::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
    //get the current pose of the robot in the fixed frame
    geometry_msgs::PoseStamped robot_pose;
    if(!costmap_ros_->getRobotPose(robot_pose)){
      ROS_ERROR("Can't get robot pose");
      geometry_msgs::Twist empty_twist;
      cmd_vel = empty_twist;
      return false;
    }

    executing_goal = true;
    std_msgs::Float64 state_msg;

    //ROS_INFO("Robot is w.r.t to %s",robot_pose.header.frame_id.c_str());
    //ROS_INFO("Path is w.r.t to %s",global_plan_world_coordinates[0].header.frame_id.c_str());

    int closestPoint = getClosestPoint(global_plan_world_coordinates, robot_pose.pose);

    auto closestPose = global_plan_world_coordinates[closestPoint];

    current_closest_point = global_plan_world_coordinates[global_plan_world_coordinates.size() -1];

    ROS_INFO("Robot Coordinates: x:%f y:%f yaw: %f", 
                robot_pose.pose.position.x, robot_pose.pose.position.y, tf2::getYaw(robot_pose.pose.orientation));
    ROS_INFO("Closest Point Coordinates: x:%f y:%f yaw: %f",
                closestPose.pose.position.x, closestPose.pose.position.y, tf2::getYaw(closestPose.pose.orientation));
    
    double yaw_difference = tf2::getYaw(robot_pose.pose.orientation) - tf2::getYaw(closestPose.pose.orientation);
    tf2::Quaternion robot_quat;
    tf2::Quaternion close_quat;

    tf2::fromMsg(robot_pose.pose.orientation, robot_quat);
    tf2::fromMsg(closestPose.pose.orientation, close_quat);

    ROS_INFO("Difference: %f Minimum Angle:%f", yaw_difference, tf2::angleShortestPath(robot_quat, close_quat));

    geometry_msgs::Twist diff = diff2D(robot_pose.pose, 
              global_plan_world_coordinates[global_plan_world_coordinates.size() -1].pose);
    ROS_INFO("PoseFollower: Goal Point diff %f %f ==> %f", diff.linear.x, diff.linear.y, diff.angular.z);

    int next_point = (closestPoint == global_plan_world_coordinates.size() - 1) ? (closestPoint - 1) : (closestPoint + 1);
    double curve;

    if(next_point > closestPoint){
      curve = slope(global_plan_world_coordinates[closestPoint], global_plan_world_coordinates[next_point]);
    }else{
      curve = slope(global_plan_world_coordinates[next_point], global_plan_world_coordinates[closestPoint]);
    }

    
    geometry_msgs::Twist command_signal;

    //command_signal.linear.x = distBetweenPoints(robot_pose.pose, 
            //global_plan_world_coordinates[global_plan_world_coordinates.size() -1].pose) * K_linear;
    
    command_signal.linear.x = linear_control_effort;
    state_msg.data = distBetweenPoints(robot_pose.pose, closestPose.pose) * sign(robot_pose.pose.position.y - closestPose.pose.position.y);
    steer_state_pub.publish(state_msg);
    //auto return_signal = ros::topic::waitForMessage<std_msgs::Float64>(steer_control_effort_topic);
    command_signal.angular.z = steer_control_effort;
    
    //max_vel_th_ = command_signal.linear.x * tan(max_turn_angle) / wheelbase;

    geometry_msgs::Twist limit_vel = limitTwist(command_signal);

    geometry_msgs::Twist test_vel = limit_vel;
    bool legal_traj = collision_planner_.checkTrajectory(test_vel.linear.x, test_vel.linear.y, test_vel.angular.z, true);

    double scaling_factor = 1.0;
    double ds = scaling_factor / samples_;

    //let's make sure that the velocity command is legal... and if not, scale down
    if(!legal_traj){
      for(int i = 0; i < samples_; ++i){
        test_vel.linear.x = limit_vel.linear.x * scaling_factor;
        test_vel.linear.y = limit_vel.linear.y * scaling_factor;
        test_vel.angular.z = limit_vel.angular.z * scaling_factor;
        test_vel = limitTwist(test_vel);
        if(collision_planner_.checkTrajectory(test_vel.linear.x, test_vel.linear.y, test_vel.angular.z, false)){
          legal_traj = true;
          break;
        }
        scaling_factor -= ds;
      }
    }

    if(!legal_traj){
      ROS_ERROR("Not legal (%.2f, %.2f, %.2f)", limit_vel.linear.x, limit_vel.linear.y, limit_vel.angular.z);
      geometry_msgs::Twist empty_twist;
      cmd_vel = empty_twist;
      return false;
    }

    cmd_vel = command_signal;

    bool in_goal_position = false;
    if(fabs(diff.linear.x) <= tolerance_trans_ &&
          fabs(diff.linear.y) <= tolerance_trans_ &&
	  fabs(diff.angular.z) <= tolerance_rot_)
    {
        ROS_INFO("Reached goal: %d", current_waypoint_);
        executing_goal = false;
        in_goal_position = true;
	//cmd_vel.linear.x = 0.0;
        //cmd_vel.angular.z = 0.0;
    }

    //if we're not in the goal position, we need to update time
    if(!in_goal_position)
      goal_reached_time_ = ros::Time::now();

    //check if we've reached our goal for long enough to succeed
    if(goal_reached_time_ + ros::Duration(tolerance_timeout_) < ros::Time::now()){
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
      ROS_WARN("Long Enough To Succeed");
    }

    cmd_vel.linear.y = 0.0;
    return true;
  }

  bool PoseFollower::setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan){
    ROS_WARN_NAMED("Pose Follower","New Set Plan issued");
    current_waypoint_ = 0;
    goal_reached_time_ = ros::Time::now();
    global_plan_world_coordinates = global_plan;
    if(!transformGlobalPlan(*tf_, global_plan, *costmap_ros_, costmap_ros_->getGlobalFrameID(), global_plan_)){
      ROS_ERROR("Could not transform the global plan to the frame of the controller");
      return false;
    }

    ROS_INFO("global plan size: %lu", global_plan_.size());
    publishPlan(global_plan_, global_plan_pub_);
    return true;
  }

  bool PoseFollower::isGoalReached(){
    return goal_reached_time_ + ros::Duration(tolerance_timeout_) < ros::Time::now();
  }

  

  geometry_msgs::Twist PoseFollower::diff2D(const geometry_msgs::Pose& pose1_msg,
                                            const geometry_msgs::Pose& pose2_msg)
  {
    tf2::Transform pose1, pose2;
    tf2::convert(pose1_msg, pose1);
    tf2::convert(pose2_msg, pose2);
    geometry_msgs::Twist res;
    tf2::Transform diff = pose2.inverse() * pose1;
    res.linear.x = diff.getOrigin().x();
    res.linear.y = diff.getOrigin().y();
    res.angular.z = tf2::getYaw(diff.getRotation());

    

    //in the case that we're not rotating to our goal position and we have a non-holonomic robot
    //we'll need to command a rotational velocity that will help us reach our desired heading
    
    //we want to compute a goal based on the heading difference between our pose and the target
    double yaw_diff = headingDiff(pose1.getOrigin().x(), pose1.getOrigin().y(), 
        pose2.getOrigin().x(), pose2.getOrigin().y(), tf2::getYaw(pose2.getRotation()));

    

    //compute the desired quaterion
    tf2::Quaternion rot_diff;
    rot_diff.setRPY(0.0, 0.0, yaw_diff);
    tf2::Quaternion rot = pose2.getRotation() * rot_diff;
    tf2::Transform new_pose = pose1;
    new_pose.setRotation(rot);

    diff = pose2.inverse() * new_pose;
    res.linear.x = diff.getOrigin().x();
    res.linear.y = diff.getOrigin().y();
    res.angular.z = tf2::getYaw(diff.getRotation());
    return res;
  }


  geometry_msgs::Twist PoseFollower::limitTwist(const geometry_msgs::Twist& twist)
  {
    geometry_msgs::Twist res = twist;

    if (fabs(res.angular.z) > max_vel_th_) res.angular.z = max_vel_th_ * sign(res.angular.z);
    if (fabs(res.angular.z) < min_vel_th_) res.angular.z = min_vel_th_ * sign(res.angular.z);
    if (std::isnan(res.linear.x))
        res.linear.x = 0.0;
    if (std::isnan(res.linear.y))
        res.linear.y = 0.0;

    if (fabs(res.linear.x) > max_vel_th_) res.linear.x = max_vel_lin_ * sign(res.linear.x);
    if (fabs(res.linear.x) < min_vel_th_) res.linear.x = min_vel_lin_ * sign(res.linear.x);
    

    ROS_INFO("Angular command: %f", res.angular.z);
    ROS_INFO("Linear command: %f", res.linear.x);
    return res;
  }

  bool PoseFollower::transformGlobalPlan(const tf2_ros::Buffer& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan,
      const costmap_2d::Costmap2DROS& costmap, const std::string& global_frame,
      std::vector<geometry_msgs::PoseStamped>& transformed_plan){
    const geometry_msgs::PoseStamped& plan_pose = global_plan[0];

    transformed_plan.clear();
    
    try{
      if (global_plan.empty())
      {
        ROS_ERROR("Recieved plan with zero length");
        return false;
      }

      geometry_msgs::TransformStamped transform;
      transform = tf.lookupTransform(global_frame, ros::Time(),
                                     plan_pose.header.frame_id, plan_pose.header.stamp,
                                     plan_pose.header.frame_id);
      tf2::Stamped<tf2::Transform> tf_transform;
      tf2::convert(transform, tf_transform);

      tf2::Stamped<tf2::Transform> tf_pose;
      geometry_msgs::PoseStamped newer_pose;
      //now we'll transform until points are outside of our distance threshold
      for(unsigned int i = 0; i < global_plan.size(); ++i){
        const geometry_msgs::PoseStamped& pose = global_plan[i];
        tf2::convert(pose, tf_pose);
        tf_pose.setData(tf_transform * tf_pose);
        tf_pose.stamp_ = tf_transform.stamp_;
        tf_pose.frame_id_ = global_frame;
        tf2::toMsg(tf_pose, newer_pose);

        transformed_plan.push_back(newer_pose);
      }
    }
    catch(tf2::LookupException& ex) {
      ROS_ERROR("No Transform available Error: %s\n", ex.what());
      return false;
    }
    catch(tf2::ConnectivityException& ex) {
      ROS_ERROR("Connectivity Error: %s\n", ex.what());
      return false;
    }
    catch(tf2::ExtrapolationException& ex) {
      ROS_ERROR("Extrapolation Error: %s\n", ex.what());
      if (!global_plan.empty())
        ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

      return false;
    }

    return true;
  }
};
