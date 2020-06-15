#ifndef AAIP_POINT_HPP
#define AAIP_POINT_HPP

/* Struct to store a 2d point/vector using floats */
struct Point {
	// X, Y values
	float x, y;
	
	// Constructors
	Point() :
			x(0),
			y(0)
	{}
	Point(float x, float y) :
			x(x),
			y(y)
	{}
	
	// Comparison operators
	bool operator ==(const Point& other) const {
		return this->x == other.x && this->y == other.y;
	}

	bool operator !=(const Point& other) const {
		return this->x != other.x || this->y != other.y;
	}
	
	// Check if this point is valid == does not contain NaN's
	bool isValid() const {
		return !isnan(this->x) && !isnan(this->y);
	}
};

// Hard define an invalid point which can be used to signal an error through the return type
#define INVALID_POINT Point(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN())

#endif //AAIP_POINT_HPP
