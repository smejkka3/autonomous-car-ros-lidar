#ifndef SRC_RACINGCONTROL_HPP
#define SRC_RACINGCONTROL_HPP

#include <ros/ros.h>
#include "ros/console.h"
#include <tf/transform_listener.h>

#include <tuple>
#include <string>
#include <vector>
#include <track_detection/TrackMsg.h>


class RacingControl {
public:
	RacingControl();
	~RacingControl() = default;

private:
	// Ros Stuff - Used for communication
	ros::NodeHandle nodeHandle;
	ros::Subscriber trackSubscriber;
	ros::Subscriber cmdVelPlannerSubscriber;
	ros::Publisher goalPublisher;
	ros::Publisher cmdVelPublisher;
	tf::TransformListener tfListener;
	ros::Timer updateTimer;
	
	// Configuration	
	// Absolute Value of the maximum allowed steering value
	double maxSteeringAngle;

	// Absolute Value of the minimum allowed linear speed
	double minSpeed;

	// Absolute Value of the maximum allowed linear speed
	double maxSpeed;

	// Minimum/Maximum linear speed values for the different modes. These are loaded from the launch file and are then used when switching to the respective mode
	double minSpeedExploration;
	double maxSpeedExploration;
	double minSpeedRacing;
	double maxSpeedRacing;
	
	// Should a brake command be published when the incoming /cmd_vel message has a linear speed == 0
	bool breakWhenZero;
	
	// Should the simulation or real car algorithms be used?
	bool isSimulation;
	
	// Should the generated twist messages be printed to the console?
	bool printCmd;
	
	// Racing mode
	enum class Mode{Exploration, Racing, Stop};
	Mode mode;
	
	// Sets the current mode and updates the used speed parameters
	void setMode(Mode newMode);
	
	// Check if the exploration lap has been completed and check if a new goal should be published
	void update();
	
	// Optimal Track Saving
	// Maximum distance after which a new track pose should be recorded
	float maxDistanceNewTrackPoint;
	
	// Vector to hold all recorded track poses
	std::vector<geometry_msgs::Pose> trackPoints;
	
	// add track pose to the saved track poses if it is not already contained (another one is too near of it)
	void appendToTrackData(geometry_msgs::Pose pose);
	
	// Callbacks - Used by the Timers or Subscriber functions
	void trackUpdateCallback(const track_detection::TrackMsg::ConstPtr& msg);
	void cmdVelPlannerUpdateCallback(const geometry_msgs::Twist::ConstPtr& msg);
	void updateCallback(const ros::TimerEvent& event);
	
	// Cached last exploration target received
	geometry_msgs::Pose lastReceivedExplorationTarget;
	
	// Goal Publishing
	// Publish a new goal (only if it is different from the old goal)
	void publishGoal(geometry_msgs::Pose goal);
	
	// Minimum distance a new goal must differ from the old one
	double minGoalDistance = 0.1;
	
	// Cached last published goal
	geometry_msgs::Pose lastPublishedGoal;
	
	// Goal message sequence number
	unsigned int goalSeq;
	
	// Command Calculation - Create the Twist Message based on the input from the local planner. Different implementation for siumulation and real car possible
	geometry_msgs::Twist controlSimulation(const geometry_msgs::Twist::ConstPtr& input);
	geometry_msgs::Twist controlReal(const geometry_msgs::Twist::ConstPtr& input);
	
	// Find a suitable raacing target - not used since this does not improve the trajectory generated
	geometry_msgs::Pose findRacingTarget(geometry_msgs::Pose robotPose);
	
	// Helper
	// Get the currents robot pose from the tf tree
	geometry_msgs::Pose getRobotPose();
	
	// Get distance between to posed on xy plane
	static double getDistance(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b);
	
	// Limit value to the interval between min/max
	static double clamp(double value, double min, double max);
};

#endif //SRC_RACINGCONTROL_HPP
