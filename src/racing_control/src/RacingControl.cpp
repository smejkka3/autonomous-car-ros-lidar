#include "racing_control/RacingControl.hpp"

#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Twist.h>
#include <chrono>
#include <cmath>

const double EulerConstant = std::exp(1.0);

RacingControl::RacingControl() :
	goalSeq(0)
{
	// Init previous variables with default values
	lastPublishedGoal.position.x = -999;
	lastPublishedGoal.position.y = -999;
	
	lastReceivedExplorationTarget.position.x = 0;
	lastReceivedExplorationTarget.position.y = 0;
	lastReceivedExplorationTarget.orientation.x = 0;
	lastReceivedExplorationTarget.orientation.y = 0;
	lastReceivedExplorationTarget.orientation.z = 0;
	lastReceivedExplorationTarget.orientation.w = 1;
	
	// Create private handle for configuration
	ros::NodeHandle private_handle("~");

	maxSteeringAngle = private_handle.param("maxSteeringAngle", 0.5f);
	minSpeedExploration = private_handle.param("minSpeedExploration", 0.35f);
	maxSpeedExploration = private_handle.param("maxSpeedExploration", 0.85f);
	minSpeedRacing = private_handle.param("minSpeedRacing", 0.6f);
	maxSpeedRacing = private_handle.param("maxSpeedRacing", 0.85f);

	breakWhenZero = private_handle.param("breakWhenZero", true);

	isSimulation = private_handle.param("isSimulation", false);
	printCmd = private_handle.param("printCmd", false);

	maxDistanceNewTrackPoint = private_handle.param("maxDistanceNewTrackPoint", 0.9f);

	trackSubscriber = nodeHandle.subscribe<track_detection::TrackMsg>("track", 1, boost::bind(&RacingControl::trackUpdateCallback, this, _1));
	cmdVelPlannerSubscriber = nodeHandle.subscribe<geometry_msgs::Twist>("cmd_vel_planner", 1, boost::bind(&RacingControl::cmdVelPlannerUpdateCallback, this, _1));
	goalPublisher = nodeHandle.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1, true);
	cmdVelPublisher = nodeHandle.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);

	updateTimer = nodeHandle.createTimer(ros::Duration(0.05f), boost::bind(&RacingControl::updateCallback, this, _1));

	setMode(Mode::Exploration);
}

void RacingControl::update() {
	geometry_msgs::Pose robotPose = getRobotPose();

	// Check if we finished the exploration run
	if(mode == Mode::Exploration && robotPose.position.z >= 0.0 && trackPoints.size() >= 8 && getDistance(robotPose, trackPoints.front()) <= 0.8) {
		setMode(Mode::Racing);
	}
	
	// Record track data while in exploration run
	if(mode == Mode::Exploration) {
		appendToTrackData(robotPose);
	}
	
	// Publish goal
	if(mode == Mode::Exploration) {
		publishGoal(lastReceivedExplorationTarget);
	} else if(mode == Mode::Racing) {
		publishGoal(lastReceivedExplorationTarget);
		//publishGoal(findRacingTarget(robotPose));
	}
}

void RacingControl::updateCallback(const ros::TimerEvent& event) {
	update();
}

void RacingControl::trackUpdateCallback(const track_detection::TrackMsg::ConstPtr& msg) {
	lastReceivedExplorationTarget = msg->explorationTarget;

	// Reset Timer
	updateTimer.stop();
	updateTimer.start();
	
	update();
}

geometry_msgs::Pose RacingControl::findRacingTarget(geometry_msgs::Pose robotPose) {
	// Find nearest track marker
	double nearestMarkerDistance = std::numeric_limits<double>::max();
	bool foundPoint = false;
	unsigned long index = 0;
	
	for(int i = 0; i < trackPoints.size(); i++) {
		double distance = getDistance(robotPose, trackPoints[i]);
		
		if(distance <= nearestMarkerDistance) {
			nearestMarkerDistance = distance;
			index = i;
			foundPoint = true;
		}
	}
	
	if(!foundPoint) {
		ROS_ERROR("Could not find any track marker!");
		return robotPose;
	}
	
	int lookahead = (int) std::round((double)trackPoints.size() * 0.3);
	while(lookahead > 0) {
		index = (index + 1) % trackPoints.size();
		lookahead--;
	}

	return trackPoints[index];
}

void RacingControl::cmdVelPlannerUpdateCallback(const geometry_msgs::Twist::ConstPtr& input) {
	geometry_msgs::Twist output;

	if(mode == Mode::Stop) {
		output.linear.x = 0.0;
		output.angular.z = 0.0;
	} else {
		if(isSimulation) {
			output = controlSimulation(input);
		} else {
			output = controlReal(input);
		}
	}

	if(printCmd) {
		ROS_INFO("Speed: %f, Angle: %f", output.linear.x, output.angular.z);
	}

	cmdVelPublisher.publish(output);
}

void RacingControl::publishGoal(geometry_msgs::Pose goal) {
	if(getDistance(lastPublishedGoal, goal) < minGoalDistance) {
		return;
	}
	lastPublishedGoal = goal;
	
	geometry_msgs::PoseStamped msg;
	msg.header.seq = goalSeq++;
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = "map";
	msg.pose = goal;
	
	goalPublisher.publish(msg);
	ROS_INFO("Published Goal %f/%f", msg.pose.position.x, msg.pose.position.y);
}

void RacingControl::appendToTrackData(geometry_msgs::Pose pose) {
	// Check if we already had this position
	for(geometry_msgs::Pose existingPose : trackPoints) {
		if(getDistance(pose, existingPose) < maxDistanceNewTrackPoint) {
			// Pose already in list
			return;
		}
	}

	// Pose not in list -> append
	trackPoints.push_back(pose);
}

geometry_msgs::Twist RacingControl::controlSimulation(const geometry_msgs::Twist::ConstPtr& input) {
	double angle = clamp(input->angular.z, -maxSteeringAngle, maxSteeringAngle);

	double speed;
	if(input->linear.x < 0.0) {
		speed = 0.0;
	} else {
		// Calculate speed based on steering angle. The sharper the turn the smaller the speed
		speed = std::pow(EulerConstant, -std::abs(angle)) * maxSpeed;
	}

	if(!breakWhenZero || speed > 0.0) {
		speed = clamp(speed, minSpeed, maxSpeed);
	}

	geometry_msgs::Twist output;
	output.linear.x = speed;
	output.angular.z = angle;
	return output;
}

geometry_msgs::Twist RacingControl::controlReal(const geometry_msgs::Twist::ConstPtr& input) {
	double angle = clamp(input->angular.z, -maxSteeringAngle, maxSteeringAngle);

	double speed;
	if(input->linear.x < 0.0) {
		speed = 0.0;
	} else {
		// Calculate speed based on steering angle. The sharper the turn the smaller the speed
		speed = std::pow(EulerConstant, -std::abs(angle)) * maxSpeed;
	}

	if(!breakWhenZero || speed > 0.0) {
		speed = clamp(speed, minSpeed, maxSpeed);
	}

	geometry_msgs::Twist output;
	output.linear.x = speed;
	output.angular.z = angle;
	return output;
}

double RacingControl::getDistance(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b) {
	return std::sqrt(std::pow(a.position.x - b.position.x, 2) + std::pow(a.position.y - b.position.y, 2));
}

double RacingControl::clamp(double value, double min, double max) {
	return std::max(std::min(value, max), min);
}

void RacingControl::setMode(RacingControl::Mode newMode) {
	this->mode = newMode;
	
	switch(newMode) {
		case Mode::Exploration:
			minSpeed = minSpeedExploration;
			maxSpeed = maxSpeedExploration;
			ROS_INFO("Changed Mode to EXPLORATION!");
			break;
		case Mode::Racing:
			minSpeed = minSpeedRacing;
			maxSpeed = maxSpeedRacing;
			ROS_INFO("===================\nChanged Mode to RACING, recorded %d track markers!\n===================", (int) trackPoints.size());
			break;
		case Mode::Stop:
			minSpeed = 0;
			maxSpeed = 0;
			ROS_INFO("Changed Mode to STOP!");
			break;
	}
}

geometry_msgs::Pose RacingControl::getRobotPose() {
	tf::StampedTransform transform;
	try {
		tfListener.lookupTransform("/map", "/base", ros::Time(0), transform);
	} catch(tf::TransformException& ex) {
		ROS_WARN("Cant get current robot position: %s", ex.what());
		geometry_msgs::Pose errorPose;
		errorPose.position.z = -1.0;
		return errorPose;
	}

	// Add current robot position to track data
	geometry_msgs::Pose robotPose;
	robotPose.position.x = transform.getOrigin().getX();
	robotPose.position.y = transform.getOrigin().getY();
	robotPose.position.z = 0.0;
	robotPose.orientation.x = transform.getRotation().x();
	robotPose.orientation.y = transform.getRotation().y();
	robotPose.orientation.z = transform.getRotation().z();
	robotPose.orientation.w = transform.getRotation().w();
	
	return robotPose;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "racing_control");

	RacingControl racingControl;

	ros::spin();

	return 0;
}