#include "track_detection/TrackDetector.hpp"

#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Twist.h>
#include <memory>
#include <chrono>
#include <track_detection/TrackMsg.h>

#define EPS 0.0001f

TrackDetection::TrackDetection() :
		prevTrackMarkerCount(0),
		prevExplorationTargetMarkerCount(0)
{
	// Create private handle for configuration
	ros::NodeHandle private_handle("~");

	// These settings override the ones from the header file!!
	explorationHeading = private_handle.param("explorationHeading", 1.5f);
	explorationRobotPositionOffset = private_handle.param("explorationRobotPositionOffset", 0.75f);
	maxExplorationConeDistance = private_handle.param("maxExplorationConeDistance", 2.5f);
	defaultStartLineRadius = private_handle.param("defaultStartLineRadius", 1.25f);
	maxConeDistance = private_handle.param("maxConeDistance", 1.5f);

	// Set up advertisements and subscriptions
	mapSubscriber = nodeHandle.subscribe<nav_msgs::OccupancyGrid>("map", 1, boost::bind(&TrackDetection::mapUpdateCallback, this, _1));
	mapPublisher = nodeHandle.advertise<nav_msgs::OccupancyGrid>("map_obstacles", 1, true);
	trackPublisher = nodeHandle.advertise<track_detection::TrackMsg>("track", 1, true);
	trackVisPublisher = nodeHandle.advertise<visualization_msgs::MarkerArray>("track_vis", 1, true);
	conesVisPublisher = nodeHandle.advertise<visualization_msgs::MarkerArray>("cones_vis", 1, true);
	explorationVisPublisher = nodeHandle.advertise<visualization_msgs::MarkerArray>("exploration_vis", 1, true);

	explorationUpdateTimer = nodeHandle.createTimer(ros::Duration(0.1f), boost::bind(&TrackDetection::explorationUpdateCallback, this, _1));

	// Setup initial exploration target
	leftExplorationCone = Point(0, +defaultStartLineRadius);
	rightExplorationCone = Point(0, -defaultStartLineRadius);
	explorationTarget = calculateExplorationTarget();
}

void TrackDetection::mapUpdateCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
	ROS_DEBUG("Got Map %d (%dx%d)", msg->header.seq, msg->info.width, msg->info.height);

	map = std::make_unique<Map>(msg);

	map->match();
	cones = map->getCones();
	updateTrack();
	updateExplorationData();

	publishMapWithObstacles(msg);
	publishTrackData();

	publishTrackVisualization();
	publishConeVisualization();
	publishExplorationTargetVisualization();

	// Reset explorationUpdateTimer
	explorationUpdateTimer.stop();
	explorationUpdateTimer.start();
}

void TrackDetection::explorationUpdateCallback(const ros::TimerEvent& event) {
	updateExplorationData();

	publishTrackData();
	publishExplorationTargetVisualization();
}

void TrackDetection::updateTrack() {
	leftPoints.clear();
	rightPoints.clear();

	updateStartLine();

	if(cones.empty()) {
		return;
	}

	auto subgraphs = findConnectedSubgraphs(cones);
	ROS_INFO("Found %zu subgraphs!", subgraphs.size());
	pruneSmallSubgraphs(subgraphs);
	ROS_INFO("Pruned small subgraphs, %zu remaining", subgraphs.size());

	// Assume starting position 0/0, orientation in +X
	std::vector<Point> leftGraph, rightGraph;
	float leftMinDistance, rightMinDistance;
	tie(leftGraph, leftMinDistance) = findClosestSubgraph(subgraphs, Point(0, +defaultStartLineRadius));
	tie(rightGraph, rightMinDistance) = findClosestSubgraph(subgraphs, Point(0, -defaultStartLineRadius));

	if(leftGraph.front() == rightGraph.front()) {
		if(leftMinDistance < rightMinDistance) {
			rightGraph.clear();
		} else {
			leftGraph.clear();
		}

		if(subgraphs.size() == 1) {
			// We only have one subgraph, check which is the correct side
			ROS_WARN("Only detected one side!");
		} else {
			// We have >2 subgraphs but got the same for left and right
			ROS_WARN("Detected the same side 2x although more than 2 subgraphs are available!");
		}
	}

	if(!leftGraph.empty()) {
		leftPoints = leftGraph;
	}
	if(!rightGraph.empty()) {
		rightPoints = rightGraph;
	}

	leftWalls = getWalls(leftPoints);
	rightWalls = getWalls(rightPoints);

	ROS_INFO("Detected tracks with size left: %zu, right: %zu", leftPoints.size(), rightPoints.size());
}

std::vector<std::vector<Point>> TrackDetection::findConnectedSubgraphs(std::vector<Point> points) {
	std::vector<std::vector<Point>> subgraphs;

	while(!points.empty()) {
		std::vector<Point> graph;
		std::vector<Point> openList;

		openList.push_back(points.back());
		points.pop_back();

		while(!openList.empty()) {
			Point curr = openList.back();
			openList.pop_back();

			graph.push_back(curr);

			auto iter = points.begin();
			while(iter != points.end()) {
				if(isInRange(curr, *iter)) {
					openList.push_back(*iter);
					iter = points.erase(iter);
				} else {
					iter++;
				}
			}
		}

		subgraphs.push_back(graph);
	}

	return subgraphs;
}

void TrackDetection::pruneSmallSubgraphs(std::vector<std::vector<Point>>& graphs) {
	int minSize = 2;
	int subgraphsWithMinSize = 0;

	for(const std::vector<Point>& g : graphs) {
		if(g.size() >= minSize) {
			subgraphsWithMinSize++;
		}
	}

	// If we have enough subgraphs with not just one cone -> prune the rest
	if(subgraphsWithMinSize >= 2) {
		auto iter = graphs.begin();

		while(iter != graphs.end()) {
			if((*iter).size() < minSize) {
				iter = graphs.erase(iter);
			} else {
				iter++;
			}
		}
	}
}

std::tuple<std::vector<Point>, float> TrackDetection::findClosestSubgraph(const std::vector<std::vector<Point>>& graphs, Point point) {
	ROS_ASSERT(!graphs.empty());

	int closestSubgraph = 0;
	float minDistance = std::numeric_limits<float>::max();

	for(int i = 0; i < graphs.size(); i++) {
		float minDistanceForSubgraph = std::numeric_limits<float>::max();
		for(const Point& p : graphs[i]) {
			minDistanceForSubgraph = std::min(getDistance(point, p), minDistanceForSubgraph);
		}

		if(minDistanceForSubgraph < minDistance) {
			minDistance = minDistanceForSubgraph;
			closestSubgraph = i;
		}
	}

	return std::make_tuple(graphs[closestSubgraph], minDistance);
}

Point TrackDetection::findClosestNeighbour(Point origin, const std::vector<Point>& candidates, const std::vector<Point>& visited, float maxDistance) {
	float minDistance = std::numeric_limits<float>::max();

	Point closestNeighbour = INVALID_POINT;
	for(const Point& neighbour : candidates) {
		if(!neighbour.isValid()) {
			continue;
		}

		float dist = getDistance(origin, neighbour);
		if(dist <= maxDistance && dist < minDistance && dist >= 0.0001f && !containsPoint(visited, neighbour)) {
			minDistance = dist;
			closestNeighbour = neighbour;
		}
	}

	return closestNeighbour;
}

std::vector<std::pair<Point, Point>> TrackDetection::getWalls(const std::vector<Point>& points) {
	std::vector<std::pair<Point, Point>> walls;

	for(const Point& p : points) {
		for(const Point& n : points) {
			if(isInRange(p, n) && p != n && !containsConnection(walls, p, n)) {
				walls.emplace_back(std::make_pair(p, n));
			}
		}
	}

	return walls;
}

void TrackDetection::updateStartLine() {
	leftStart = Point(0, +defaultStartLineRadius);
	rightStart = Point(0, -defaultStartLineRadius);
}

void TrackDetection::updateExplorationData() {
	// Get robot position
	tf::StampedTransform transform;	
	try {
		tfListener.lookupTransform("/map", "/base", ros::Time(0), transform);
	} catch(tf::TransformException& ex) {
		ROS_WARN("Cant update exploration target: %s", ex.what());
		return;
	}

	if(leftPoints.empty() || rightPoints.empty()) {
		ROS_DEBUG("Cant update exploration target: Not enough cones detected yet!");
		return;
	}

	// Move robot position forward 
	tf::Vector3 forward(explorationRobotPositionOffset, 0.f, 0.f);
	forward = tf::quatRotate(transform.getRotation(), forward);
	Point explorationBase = Point((float)(transform.getOrigin().getX() + forward.getX()), (float)(transform.getOrigin().getY() + forward.getY()));
	
	// Find closest cones to this calculated position
	Point leftExplorationConeTemp = findClosestNeighbour(explorationBase, leftPoints, std::vector<Point>(), maxExplorationConeDistance);
	Point rightExplorationConeTemp = findClosestNeighbour(explorationBase, rightPoints, std::vector<Point>(), maxExplorationConeDistance);

	if(!leftExplorationConeTemp.isValid() || !rightExplorationConeTemp.isValid()) {
		ROS_WARN("Cant update exploration target: Did not find suitable cone nearby!");
		return;
	}
	
	leftExplorationCone = leftExplorationConeTemp;
	rightExplorationCone = rightExplorationConeTemp;

	// Update exploration target based on these detected cones
	geometry_msgs::Pose explorationTargetTemp = calculateExplorationTarget();
	if(!isnan(explorationTargetTemp.position.x) && !isnan(explorationTargetTemp.position.y)) {
		this->explorationTarget = explorationTargetTemp;
		ROS_DEBUG("Exploration target: %f/%f - %f", explorationTarget.position.x, explorationTarget.position.y, tf::getYaw(explorationTarget.orientation));
	}
}

geometry_msgs::Pose TrackDetection::calculateExplorationTarget() {
	Point p = computeCenterWithOffset(leftExplorationCone, rightExplorationCone, explorationHeading);

	// Compute direction
	Point direction = Point(rightExplorationCone.x - leftExplorationCone.x, rightExplorationCone.y - leftExplorationCone.y);

	// Normalize
	float length = std::sqrt(direction.x * direction.x + direction.y * direction.y) + EPS;
	if(std::abs(direction.x) > EPS) {
		direction.x = direction.x / length;
	}
	if(std::abs(direction.y) > EPS) {
		direction.y = direction.y / length;
	}

	// Compute quaternion rotated by 90° ccw
	direction = Point(-direction.y, direction.x);
	tf::Quaternion q = tf::createQuaternionFromYaw(std::atan2(direction.y, direction.x));

	geometry_msgs::Pose pose;
	pose.position.x = p.x;
	pose.position.y = p.y;
	pose.position.z = 0;
	pose.orientation.x = q.x();
	pose.orientation.y = q.y();
	pose.orientation.z = q.z();
	pose.orientation.w = q.w();

	return pose;
}

void TrackDetection::publishMapWithObstacles(const nav_msgs::OccupancyGrid::ConstPtr& originalMap) {
	// Start time measurement
	auto t1 = std::chrono::high_resolution_clock::now();

	nav_msgs::OccupancyGrid msg;
	msg.info = originalMap->info;
	msg.header = originalMap->header;
	msg.header.stamp = ros::Time::now();

	// Reserve space for the map data and initialize it as free
	msg.data = std::vector<int8_t>(msg.info.width * msg.info.height, 0);
	float width = 25.f;

	for(const std::pair<Point, Point>& wall : leftWalls) {
		bresenhamLineWidth(wall.first.x, wall.first.y, wall.second.x, wall.second.y, width, msg.data.data(), &msg.info);
	}
	for(const std::pair<Point, Point>& wall : rightWalls) {
		bresenhamLineWidth(wall.first.x, wall.first.y, wall.second.x, wall.second.y, width, msg.data.data(), &msg.info);
	}

	mapPublisher.publish(msg);

	// Stop time measurement
	auto t2 = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
	ROS_DEBUG("Rasterized walls in %d ms", (int) duration);
}

void TrackDetection::publishTrackData() {
	track_detection::TrackMsg msg;

	msg.explorationTarget = explorationTarget;

	trackPublisher.publish(msg);
}

void TrackDetection::bresenhamLineWidth(float x0_, float y0_, float x1_, float y1_, float width, int8_t* data, nav_msgs::MapMetaData* mapData) {
	// Taken from http://members.chello.at/~easyfilter/bresenham.html
	// Slightly adapted
	int x0 = (int) ((x0_ - mapData->origin.position.x) / mapData->resolution);
	int y0 = (int) ((y0_ - mapData->origin.position.y) / mapData->resolution);
	int x1 = (int) ((x1_ - mapData->origin.position.x) / mapData->resolution);
	int y1 = (int) ((y1_ - mapData->origin.position.y) / mapData->resolution);

	int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
	int dy = std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
	int err = dx - dy, e2, x2, y2; /* error value e_xy */
	float ed = dx + dy == 0 ? 1 : sqrt((float) (dx * dx) + (float) (dy * dy));
	float threshold = 1.f;

	for(width = (width + 1) / 2;;) {
		if(std::abs((float) std::abs(err - dx + dy) / ed - width + 1.f) >= threshold) {
			data[getIndex(x0, y0, mapData)] = 100;
		}
		e2 = err;
		x2 = x0;
		// X
		if(2 * e2 >= -dx) {
			for(e2 += dy, y2 = y0; (float) e2 < ed * width && (y1 != y2 || dx > dy); e2 += dx) {
				y2 += sy;
				if(std::abs((float) std::abs(e2) / ed - width + 1.f) >= threshold) {
					data[getIndex(x0, y2, mapData)] = 100;
				}
			}
			if(x0 == x1) {
				break;
			}
			e2 = err;
			err -= dy;
			x0 += sx;
		}
		// Y
		if(2 * e2 <= dy) {
			for(e2 = dx - e2; (float) e2 < ed * width && (x1 != x2 || dx < dy); e2 += dy) {
				x2 += sx;
				if(std::abs((float) std::abs(e2) / ed - width + 1.0) >= threshold) {
					data[getIndex(x2, y0, mapData)] = 100;
				}
			}
			if(y0 == y1) {
				break;
			}
			err += dx;
			y0 += sy;
		}
	}
}

unsigned int TrackDetection::getIndex(int x, int y, nav_msgs::MapMetaData* mapData) {
	return (unsigned int) y * mapData->width + x;
}

void TrackDetection::publishTrackVisualization() {
	visualization_msgs::MarkerArray msg;

	visualization_msgs::Marker edge;
	edge.header.frame_id = "map";
	edge.header.stamp = ros::Time::now();
	edge.action = visualization_msgs::Marker::ADD;
	edge.ns = "track_detection";
	edge.id = 0;
	edge.type = visualization_msgs::Marker::LINE_STRIP;
	edge.scale.x = 0.075;
	edge.pose.orientation.x = 0.0;
	edge.pose.orientation.y = 0.0;
	edge.pose.orientation.z = 0.0;
	edge.pose.orientation.w = 1.0;

	uint id = 0;
	geometry_msgs::Point p;

	//Start Line
	edge.color.r = 0.0;
	edge.color.g = 1.0;
	edge.color.b = 0.0;
	edge.color.a = 0.45;
	edge.points.clear();
	p.x = leftStart.x;
	p.y = leftStart.y;
	edge.points.push_back(p);
	p.x = rightStart.x;
	p.y = rightStart.y;
	edge.points.push_back(p);

	edge.id = id;
	msg.markers.push_back(edge);
	id++;

	// Left
	edge.color.r = 0.0;
	edge.color.g = 0.0;
	edge.color.b = 1.0;
	edge.color.a = 0.75;
	for(auto wall : leftWalls) {
		edge.points.clear();
		p.x = wall.first.x;
		p.y = wall.first.y;
		edge.points.push_back(p);
		p.x = wall.second.x;
		p.y = wall.second.y;
		edge.points.push_back(p);

		edge.id = id;
		msg.markers.push_back(edge);
		id++;
	}

	// Right
	edge.color.r = 1.0;
	edge.color.g = 0.0;
	edge.color.b = 0.0;
	edge.color.a = 0.75;
	for(auto wall : rightWalls) {
		edge.points.clear();
		p.x = wall.first.x;
		p.y = wall.first.y;
		edge.points.push_back(p);
		p.x = wall.second.x;
		p.y = wall.second.y;
		edge.points.push_back(p);

		edge.id = id;
		msg.markers.push_back(edge);
		id++;
	}

	// Delete unnecessary markers
	edge.action = visualization_msgs::Marker::DELETE;
	for(; id < prevTrackMarkerCount; id++) {
		edge.id = id;
		msg.markers.push_back(edge);
	}

	prevTrackMarkerCount = msg.markers.size();
	trackVisPublisher.publish(msg);
}

void TrackDetection::publishConeVisualization() {
	visualization_msgs::MarkerArray msg;

	visualization_msgs::Marker marker;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time::now();
	marker.action = visualization_msgs::Marker::ADD;
	marker.ns = "track_detection";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::SPHERE_LIST;
	marker.scale.x = 0.3;
	marker.scale.y = 0.3;
	marker.scale.z = 0.3;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	marker.color.a = 0.85;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	if(!cones.empty()) {
		for(const Point& c : cones) {
			geometry_msgs::Point p;
			p.x = c.x;
			p.y = c.y;
			p.z = 0.01f;
			marker.points.push_back(p);
		}
	} else {
		marker.action = visualization_msgs::Marker::DELETE;
	}

	msg.markers.push_back(marker);
	conesVisPublisher.publish(msg);
}

void TrackDetection::publishExplorationTargetVisualization() {
	visualization_msgs::MarkerArray msg;

	visualization_msgs::Marker marker;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time::now();
	marker.action = visualization_msgs::Marker::ADD;
	marker.ns = "track_detection";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.scale.x = 0.025;
	marker.scale.y = 0.0;
	marker.scale.z = 0.0;
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 1.0;
	marker.color.a = 0.5;
	marker.pose.position.x = 0.f;
	marker.pose.position.y = 0.f;
	marker.pose.position.z = 0.0001f;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	// exploration cones
	marker.points.clear();
	geometry_msgs::Point p;
	p.x = leftExplorationCone.x;
	p.y = leftExplorationCone.y;
	marker.points.push_back(p);
	p.x = rightExplorationCone.x;
	p.y = rightExplorationCone.y;
	marker.points.push_back(p);

	msg.markers.push_back(marker);
	marker.points.clear();

	// Target arrow
	marker.type = visualization_msgs::Marker::ARROW;
	marker.scale.x = 0.35;
	marker.scale.y = 0.07;
	marker.scale.z = 0.07;
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.5;
	marker.color.a = 1.0;
	marker.id = 1;

	marker.pose.position.x = explorationTarget.position.x;
	marker.pose.position.y = explorationTarget.position.y;
	marker.pose.orientation = explorationTarget.orientation;
	msg.markers.push_back(marker);

	// Delete unnecessary markers
	marker.action = visualization_msgs::Marker::DELETE;
	for(; marker.id < prevExplorationTargetMarkerCount; marker.id++) {
		msg.markers.push_back(marker);
	}

	prevExplorationTargetMarkerCount = marker.id;
	explorationVisPublisher.publish(msg);
}

float TrackDetection::getDistance(const Point& a, const Point& b) {
	return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

bool TrackDetection::isInRange(const Point& a, const Point& b) {
	return getDistance(a, b) <= maxConeDistance;
}

bool TrackDetection::containsPoint(const std::vector<Point>& list, const Point& a) {
	if(list.empty()) {
		return false;
	}
	return std::find(list.begin(), list.end(), a) != list.end();
}

bool TrackDetection::containsConnection(std::vector<std::pair<Point, Point>> list, const Point& a, const Point& b) {
	return std::find(list.begin(), list.end(), std::make_pair(a, b)) != list.end() || std::find(list.begin(), list.end(), std::make_pair(b, a)) != list.end();
}

Point TrackDetection::computeCenterWithOffset(Point left, Point right, float offset) {
	Point center = Point((left.x + right.x) / 2.f, (left.y + right.y) / 2.f);
	Point direction = Point(right.x - left.x, right.y - left.y);

	// Normalize
	float length = std::sqrt(direction.x * direction.x + direction.y * direction.y) + EPS;
	if(std::abs(direction.x) > EPS) {
		direction.x = direction.x / length * offset;
	}
	if(std::abs(direction.y) > EPS) {
		direction.y = direction.y / length * offset;
	}

	// Rotate ccw
	direction = Point(-direction.y, direction.x);
	return {center.x + direction.x, center.y + direction.y};
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "track_detection");

	TrackDetection trackDetector;

	ros::spin();

	return 0;
}
