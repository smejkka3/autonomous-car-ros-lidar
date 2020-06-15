#ifndef AAIP_TRACKDETECTOR_HPP
#define AAIP_TRACKDETECTOR_HPP

#include <ros/ros.h>
#include "ros/console.h"
#include <tf/transform_listener.h>

#include <tuple>
#include <string>
#include <vector>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>

// Track Detector Includes
#include "track_detection/Map.hpp"
#include "track_detection/Point.hpp"

class TrackDetection {
public:
    TrackDetection();
    ~TrackDetection() = default;

private:
    // Ros Subscribers / Publishers / Node Handles
    ros::NodeHandle nodeHandle;
    ros::Subscriber mapSubscriber;
    ros::Publisher mapPublisher;
    ros::Publisher trackPublisher;
	ros::Publisher trackVisPublisher;
	ros::Publisher conesVisPublisher;
	ros::Publisher explorationVisPublisher;
	tf::TransformListener tfListener;
	
	// Timer
	ros::Timer explorationUpdateTimer;
	
	// List of currently detected cones
	std::vector<Point> cones;
	
    /* ====== Track representation ====== */
    // The track is split up into the left and right side of detected cones. These are stored here
    std::vector<Point> leftPoints;
    std::vector<Point> rightPoints;
    
    // Cones in one group (left or right) are then connected with walls. These walls (their endpoints) are stored here
    std::vector<std::pair<Point, Point>> leftWalls;
	std::vector<std::pair<Point, Point>> rightWalls;
	
	// The point right left (ha!) of the starting position and the point right right (...) of the starting position. 
	// Only used for visualization of the starting line
    Point leftStart;
    Point rightStart;

	/* ====== Exploration ====== */
	// To facilitate exploration of the track an "interesting point" in front of the robot is constantly being computed, based on the already detected cones
	// This point can then be used for exploration, driving towards it should yield new information about the track. This "exploration point" is computed
	// using cones ahead of the robot, one left and one right, called left/right-ExplorationCone.
	Point leftExplorationCone;
	Point rightExplorationCone;
	// The already mentioned explorationTarget. Note that this also contains an direction
	geometry_msgs::Pose explorationTarget;

	// How much should the robots position be expanded ahead of it to search for the exploration cones
	float explorationRobotPositionOffset;
	// How much should the detected exploration point be extruded forward (based on its on direction) afterwards to move it further away from the robot
	float explorationHeading;
	
	// Maximum search radius while searching for the exploration cones
	float maxExplorationConeDistance;
	
	// Length of the starting line. Only for visualization
	float defaultStartLineRadius;
	
	// Recalculate the exploration cones and heading based on the robots position and the currently detected cones
	void updateExplorationData();
	
	// Calculate the explorationTarget based on the exploration cones
	geometry_msgs::Pose calculateExplorationTarget();
    
    /* ====== Track detection ====== */
    // Maximum distance at which cones are connected to belong to the same group (left or right) and as walls
    float maxConeDistance;
    
    // Recalculate the left/right walls based on the detected cones
    void updateTrack();

    // Find all connected subgraphs/groups given the input point list. The distance maximum is the maxConeDistance
    std::vector<std::vector<Point>> findConnectedSubgraphs(std::vector<Point> points);
    
    // Remove small subgraphs if we have enough large subgraphs
    static void pruneSmallSubgraphs(std::vector<std::vector<Point>>& graphs);
    
    // Given a list of graphs and a starting point, calculate the subgraph with the point closest to the given starting point. Return this closest subgraph as well as the distance
    static std::tuple<std::vector<Point>, float> findClosestSubgraph(const std::vector<std::vector<Point>>& graphs, Point point);
    
    // Find the closest neighbour point given an origin point, a list of point candidates to search, a list of already visited points which must be excluded from the search and a maximum distance
    // If no suitable point is found INVALID_POINT is returned to signal this error case
    static Point findClosestNeighbour(Point origin, const std::vector<Point>& candidates, const std::vector<Point>& visited, float maxDistance);
    
    // Compute all walls (point-point connections) inside the given point collection
    std::vector<std::pair<Point, Point>> getWalls(const std::vector<Point>& points);
    
    // Update the starting line. Only used once in the beginning
    void updateStartLine();

    // Pointer to the underlying map which handles the cone detection 
	std::unique_ptr<Map> map;

	/* ====== Callbacks ====== */
	// Called when a new map is received. Updates the underlying map with thi data and recompute cones
	void mapUpdateCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
	
	// Called when the exploration target has not been updated for a certain period of time
	void explorationUpdateCallback(const ros::TimerEvent& event);
	
    /* ====== Visualization ====== */
	// Store the number of previously published markers per topic to be able to add remove-Markers to newer messages to delete old data in rviz
    int prevTrackMarkerCount;
    int prevExplorationTargetMarkerCount;
    
    /* ====== Publish methods ======= */    
    void publishMapWithObstacles(const nav_msgs::OccupancyGrid::ConstPtr& originalMap);
    void publishTrackData();
	void publishTrackVisualization();
	void publishConeVisualization();
	void publishExplorationTargetVisualization();

	// Helper methods
	// Get distance between two points
	static inline float getDistance(const Point& a, const Point& b);
	
	// Is point a in range of point b (using maxConeDistance as limit)
	inline bool isInRange(const Point& a, const Point& b);
	
	// Does the list contain the point?
	static inline bool containsPoint(const std::vector<Point>& list, const Point& a);
	
	// Does the list of point-pairs contain the point-pair A and B?
	static inline bool containsConnection(std::vector<std::pair<Point, Point>> list, const Point& a, const Point& b);
	
	// Compute the extruded new point based on the center of the line described by left and right and the provided extrusion offset
	static inline Point computeCenterWithOffset(Point left, Point right, float offset);
	
	// Compture the bresenhams line algorithm and fill in all cells which fall inside of the line. Start is (x0, y0), end is (x1, y1), width is the line with while data is a pointer to the map data where the obstacles should be added. mapData contains the dimensions of said map.
	static inline void bresenhamLineWidth(float x0_, float y0_, float x1_, float y1_, float width, int8_t* data, nav_msgs::MapMetaData* mapData);
	
	// Get the index of the cell at position x/y inside the data array
	static inline unsigned int getIndex(int x, int y, nav_msgs::MapMetaData* mapData);
  
};

#endif // AAIP_TRACKDETECTOR_HPP
