#ifndef AAIP_MAP_HPP
#define AAIP_MAP_HPP

#include <vector>
#include <nav_msgs/OccupancyGrid.h>

#include "CellState.hpp"
#include "ConeKernel.hpp"
#include "Point.hpp"

class Map {
public:
	// Construct from occupancy grid message message
	explicit Map(const nav_msgs::OccupancyGrid::ConstPtr& msg);
	
	// Default destructor
	~Map() = default;
	
	// Runs the cone kernel over the whole image and detects likely cone positions.
	// Afterwards those likely cone positions are checked again for a minimal number in a certain area.
	// If this check succeeds a cone is detected and saved
	void match();
	
	// Return all detected cones. Requires match() to be calles beforehand
	std::vector<Point> getCones() const;

private:
	// Data describing the underlying occupancy grid
	unsigned int width, height;
	float originX, originY;
	float resolution;
	
	// The data (cells) of the used occupancy grid
	std::vector<std::vector<CellState>> cells;
	
	// Visualization
	// Should a visualization of the heatmap be exported during runtime (to heatmap.ppm)
	bool visualize = false;
	// Vector used to store the generated heatmap visualization data
	std::vector<std::vector<float>> scores;
	// Export generated scores as heatmap to heatmap.ppm
	void exportImage();

	// Cone detection
	// Maximum distance for cone candidates to be merged (in cells)
	float maxDistance = 30;
	// Minimum number of cone candidates required
	int minMatches = 3;
	// Struct used to store cone candidates
	struct ConeCandidate{
		// Position
		Point pos;
		// Number of matches in the region around this candidate (region with radius maxDistance)
		int matches;
		ConeCandidate(float x, float y) :
				pos(x ,y),
				matches(1)
		{}
	};
	
	// The stored cone candidates
	std::vector<ConeCandidate> coneCandidates;
	// The cone kernel used to detect these cone candidates
	ConeKernel kernel;
	
	// Add cone candidate at position x/y if there is not already one there. In that case increase the count of the first one
	void addConeCandidate(float x, float y);
	
	// Merge all cone candidates to one cone if the minimum number of cone candidates sufficient
	void mergeConeCandidates();

	// Helper functions
	static float getDistance(float x1, float y1, float x2, float y2);
	
};


#endif //AAIP_MAP_HPP
