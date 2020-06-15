#include "track_detection/Map.hpp"
#include "track_detection/ConeKernel.hpp"
#include <ros/assert.h>
#include <chrono>

Map::Map(const nav_msgs::OccupancyGrid::ConstPtr& msg) :
	width(msg->info.width),
	height(msg->info.height),
	originX(msg->info.origin.position.x),
	originY(msg->info.origin.position.y),
	resolution(msg->info.resolution)
{
	ROS_ASSERT(width > 0 && height > 0 && resolution > 0);
	
	// Construct Cells vector from occupancy grid message
	cells = std::vector<std::vector<CellState>>(width, std::vector<CellState>(height, CellState::UNKNOWN));
	for(unsigned int x = 0; x < width; x++) {
		for(unsigned int y = 0; y < height; y++) {
			cells[x][y] = static_cast<CellState>(msg->data[y * width + x]);
		}
	}
}

void Map::match() {
	// Only for visualization
	if(visualize) {
		scores = std::vector<std::vector<float>>(width, std::vector<float>(height, 0));
	}
	
	// Start time measurement
	auto t1 = std::chrono::high_resolution_clock::now();
	
	// Clear previous cone candidates
	coneCandidates.clear();
	
	// Limit search region to avid out of border memory access
	unsigned int r = kernel.getMaxAccessedRadius() * 2; // We don't need the border anyway
	
	// Loop over every pixel and apply the cone kernel to compute if a cone is likely at this position.
	// If no cone candidate was detected skip the next pixel to sped things up. If no cone candidate was detected in a whole line skip the next one
	for(unsigned int x = r; x < width - r; x++) {
		bool canSkipNextLine = true;
		for(unsigned int y = r; y < height - r; y++) {
			bool result = kernel.apply(cells, x, y);
			
			if(result) {
				addConeCandidate((float) x, (float) y);
				canSkipNextLine = false;
			} else {
				y++;
			}
			
			if(visualize && result) {
				scores[x][y] = result;	
			}			
		}
		
		if(canSkipNextLine) {
			x++;
		}
	}

	mergeConeCandidates();	

	// Stop time measurement
	auto t2 = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
	ROS_INFO("Detected %d cones in %d ms", (int) coneCandidates.size(), (int) duration);
	
	if(visualize) {
		exportImage();
	}
}

std::vector<Point> Map::getCones() const {
	std::vector<Point> cones;
	cones.reserve(coneCandidates.size());

	for(const ConeCandidate& c : coneCandidates) {
		// Transform into map frame
		float x = originX + (c.pos.x * resolution);
		float y = originY + (c.pos.y * resolution);
		cones.emplace_back(x, y);
	}

	return cones;
}

float Map::getDistance(float x1, float y1, float x2, float y2) {
	return std::sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
}

void Map::addConeCandidate(float x, float y) {
	// Check if this is a new candidate or if it can be merged with an existing one
	bool isNew = true;
	for(ConeCandidate& c : coneCandidates) {
		if(getDistance(c.pos.x, c.pos.y, x, y) <= maxDistance) {
			c.matches++;
			isNew = false;
			break;
		}
	}

	if(isNew) {
		coneCandidates.emplace_back(x, y);
	}
}

void Map::mergeConeCandidates() {
	// Only keep candidates with minimum amount of matches
	auto it = coneCandidates.begin();
	while(it != coneCandidates.end()) {
		if(it->matches < minMatches) {
			it = coneCandidates.erase(it);
		}
		else ++it;
	}
}

void Map::exportImage() {
	FILE *f = fopen("/home/vincent/uni/aaip/heatmap.ppm", "wb");
	fprintf(f, "P6\n%i %i 255\n", width - 1, height - 1);
	for (int y=(int)height - 1; y > 0; y--) {
		for (int x=0; x < width - 1; x++) {
			int s = (int) (scores[x][y] * 255.f);

			fputc(std::min(s, 255), f);
			fputc(std::max(s - 100, 0), f);
			fputc(std::max(127 - s/2, 0), f);
		}
	}
	fclose(f);
}
