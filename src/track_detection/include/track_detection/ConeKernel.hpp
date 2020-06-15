#ifndef AAIP_CONEKERNEL_HPP
#define AAIP_CONEKERNEL_HPP

#include <vector>
#include "CellState.hpp"

class ConeKernel {
public:
	// Constructor
	ConeKernel();
	
	// Applies the kernel on the given occupancy map ("cells") at the position x/y.
	// Returns true if a cone is likely at this position
	bool apply(const std::vector<std::vector<CellState>>& cells, unsigned int x, unsigned int y) const;
	
	// Returns the maximum accessed radius by this kernel. Can be used to avoid out of bound accesses 
	int getMaxAccessedRadius() const;

private:
	// Radius of the renter region in cells
	int rCenter;
	// Min number of occupied cells in the center required
	int minOccupiedCenter;
	// Radius of the border region in cells
	int rBorder;
	// Max number of occupied cells allowed on the border
	int maxOccupiedBorder;
	// Max number of unknown cells allowed on the border
	int maxUnknownBorder;
	
};


#endif //AAIP_CONEKERNEL_HPP
