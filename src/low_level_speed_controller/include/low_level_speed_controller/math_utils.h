#ifndef MATH_UTILS_LOW_LEVEL_CONTROLLER_H
#define MATH_UTILS_LOW_LEVEL_CONTROLLER_H

#include "geometry_msgs/Point.h"
#include "math.h"

inline double clamp_value(double a, double max, double min){
    if(a < min){
        a = min;
    }

    if(a > max){
        a = max;
    }
    
    return a;
}

inline double sign(double a){
    return (a > 0.0) ? (1.0) : (-1.0);
}

inline double distance_between_points(geometry_msgs::Point a, geometry_msgs::Point b){
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

#endif
