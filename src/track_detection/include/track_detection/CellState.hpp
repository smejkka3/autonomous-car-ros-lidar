#ifndef AAIP_CELLSTATE_HPP
#define AAIP_CELLSTATE_HPP

/* Hold the Cell State values for the occupancy map*/
enum class CellState : int8_t {
	UNKNOWN = -1,
	FREE = 0,
	OCCUPIED = 100
};

#endif //AAIP_CELLSTATE_HPP
