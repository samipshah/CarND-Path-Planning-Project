#ifndef ROAD_HPP
#define ROAD_HPP

#include <src/car.hpp>
#include <assert.h>
#include <iostream>

const static unsigned int s_road_lanes = 3;
const static unsigned int s_lane_width = 4; // 4 m

unsigned int get_lane(const Car& a) { 
    if(a.m_d < 0 || a.m_d >= 12) { 
        std::cout << "What? : " << a.m_d << std::endl;
        return 0; 
    }; 
    return a.m_d/4; 
}

double get_middle_of_lane(unsigned int lane) { return 2 + lane*s_lane_width;}

#endif /* ROAD_HPP */
