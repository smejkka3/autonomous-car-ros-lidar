#pragma once

#include <type_traits>
#include <memory>

#include "low_level_speed_controller/SpeedCommandGeneratorBase.h"


template<class T>
class SpeedCommandGeneratorFactory{
    static_assert(std::is_base_of<SpeedCommandGeneratorBase, T>::value,"Class should inherit SpeedCommandInterface");
    
    public:
    static std::unique_ptr<T> create(ros::NodeHandle &n);
    

    private:
    std::unique_ptr<T> gen;

};
    
  