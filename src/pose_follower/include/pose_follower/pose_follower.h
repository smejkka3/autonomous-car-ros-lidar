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
#ifndef POSE_FOLLOWER_POSE_FOLLOWER_H_
#define POSE_FOLLOWER_POSE_FOLLOWER_H_
#include <ros/ros.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include <base_local_planner/trajectory_planner_ros.h>

namespace pose_follower {
  class PoseFollower : public nav_core::BaseLocalPlanner {
    public:
      PoseFollower();
      void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);
      bool isGoalReached();
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan);
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    private:
      inline double sign(double n){
        return n < 0.0 ? -1.0 : 1.0;
      }

      geometry_msgs::Twist diff2D(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose&  pose2);
      geometry_msgs::Twist limitTwist(const geometry_msgs::Twist& twist);
      double headingDiff(double pt_x, double pt_y, double x, double y, double heading);

      bool transformGlobalPlan(const tf2_ros::Buffer& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan,
          const costmap_2d::Costmap2DROS& costmap, const std::string& global_frame,
          std::vector<geometry_msgs::PoseStamped>& transformed_plan);

      void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
      bool stopped();
      void publishPlan(const std::vector<geometry_msgs::PoseStamped> &path, const ros::Publisher &pub);

      tf2_ros::Buffer* tf_;
      costmap_2d::Costmap2DROS* costmap_ros_;
      ros::Publisher vel_pub_;
      ros::Publisher global_plan_pub_;
      double K_trans_, K_rot_, tolerance_trans_, tolerance_rot_;
      double tolerance_timeout_;
      double max_vel_lin_, max_vel_th_;
      double min_vel_lin_, min_vel_th_;
      double min_in_place_vel_th_, in_place_trans_vel_;
      bool allow_backwards_;
      bool turn_in_place_first_;
      double max_heading_diff_before_moving_;
      bool holonomic_;
      boost::mutex odom_lock_;
      ros::Subscriber odom_sub_;
      nav_msgs::Odometry base_odom_;
      double trans_stopped_velocity_, rot_stopped_velocity_;
      ros::Time goal_reached_time_;
      unsigned int current_waypoint_; 
      std::vector<geometry_msgs::PoseStamped> global_plan_;
      base_local_planner::TrajectoryPlannerROS collision_planner_;
      int samples_;

      double K_linear;
      double K_rotational;
      double D_linear;
      double D_rot;

      double wheelbase;
      double max_turn_angle;

      ros::Subscriber control_effort_sub;
      ros::Publisher state_pub;
      ros::Publisher setpoint_pub;
      ros::Publisher enable_pub;

      std::string control_effort_topic;
      std::string state_topic;
      std::string setpoint_topic;
      std::string enable_topic;
      
      double linear_control_effort;

      ros::Subscriber steer_control_effort_sub;
      ros::Publisher steer_state_pub;
      ros::Publisher steer_setpoint_pub;
      ros::Publisher steer_enable_pub;

      std::string steer_control_effort_topic;
      std::string steer_state_topic;
      std::string steer_setpoint_topic;
      std::string steer_enable_topic;
      
      double steer_control_effort;

      std::vector<geometry_msgs::PoseStamped> global_plan_world_coordinates;

      double distBetweenPoints(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b);
      double curvature(const geometry_msgs::PoseStamped & start, const geometry_msgs::PoseStamped & end );
      double slope(const geometry_msgs::PoseStamped & start, const geometry_msgs::PoseStamped & end );
      int getClosestPoint(std::vector<geometry_msgs::PoseStamped> &global_plan, geometry_msgs::Pose pose); 

      void control_effort_callback(const std_msgs::Float64::ConstPtr &msg){
        linear_control_effort = msg->data;
      }

      void steer_control_effort_callback(const std_msgs::Float64::ConstPtr &msg){
        steer_control_effort = msg->data;
      }

      geometry_msgs::PoseStamped current_closest_point;
      bool executing_goal;
  };
};
#endif
