#include <cmath>
#include <ros/ros.h>
#include "track_detection/ConeKernel.hpp"

ConeKernel::ConeKernel() {
	ros::NodeHandle private_handle("~");

	// These settings override the ones from the header file!
	rCenter = private_handle.param("rCenter", 3);
	minOccupiedCenter = private_handle.param("minOccupiedCenter", 2);
	rBorder = private_handle.param("rBorder", 16);
	maxOccupiedBorder = private_handle.param("maxOccupiedBorder", 1);
	maxUnknownBorder = private_handle.param("maxUnknownBorder", 20);
}

bool ConeKernel::apply(const std::vector<std::vector<CellState>>& cells, unsigned int centerX, unsigned int centerY) const {
	// Early fail
	if(cells[centerX][centerY] != CellState::OCCUPIED) {
		return false;
	}

	// Center check
	int occupiedCenter = 0;
	for(int x = -rCenter; x < rCenter; x++) {
		for(int y = -rCenter; y < rCenter; y++) {
			if(cells[centerX + x][centerY + y] == CellState::OCCUPIED) {
				occupiedCenter++;
				
				if(occupiedCenter >= minOccupiedCenter) {
					goto endLoop;
				}
			}
		}
	}
	
	// Exit loop through goto. Yes this is a valid use for goto!!!
	endLoop:
	if(occupiedCenter < minOccupiedCenter) {
		return false;
	}
	
	// Check border for occupied and unknown cells. Abort as soon as maximum number is exceeded
	int occupiedBorder = 0;
	int unknownBorder = 0;
	int x;
	int y;
	
	// Top
	y = -rBorder;
	for(x = -rBorder; x < rBorder; x++) {
		if(cells[centerX + x][centerY + y] != CellState::FREE) {
			if(cells[centerX + x][centerY + y] == CellState::OCCUPIED) {
				occupiedBorder++;
			} else {
				unknownBorder++;
			}
		}
	}

	if(occupiedBorder > maxOccupiedBorder || unknownBorder > maxUnknownBorder) {
		return false;
	}

	// Bot
	y = rBorder;
	for(x = -rBorder; x < rBorder; x++) {
		if(cells[centerX + x][centerY + y] != CellState::FREE) {
			if(cells[centerX + x][centerY + y] == CellState::OCCUPIED) {
				occupiedBorder++;
			} else {
				unknownBorder++;
			}
		}
	}
	
	if(occupiedBorder > maxOccupiedBorder || unknownBorder > maxUnknownBorder) {
		return false;
	}

	// Left
	x = -rBorder;
	for(y = -rBorder+1; y < rBorder; y++) {
		if(cells[centerX + x][centerY + y] != CellState::FREE) {
			if(cells[centerX + x][centerY + y] == CellState::OCCUPIED) {
				occupiedBorder++;
			} else {
				unknownBorder++;
			}
		}
	}

	if(occupiedBorder > maxOccupiedBorder || unknownBorder > maxUnknownBorder) {
		return false;
	}

	// Right
	x = rBorder;
	for(y = -rBorder+1; y < rBorder; y++) {
		if(cells[centerX + x][centerY + y] != CellState::FREE) {
			if(cells[centerX + x][centerY + y] == CellState::OCCUPIED) {
				occupiedBorder++;
			} else {
				unknownBorder++;
			}
		}
	}

	return occupiedBorder <= maxOccupiedBorder && unknownBorder <= maxUnknownBorder;

}

int ConeKernel::getMaxAccessedRadius() const {
	return rBorder;
}

