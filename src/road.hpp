#ifndef ROAD_HPP
#define ROAD_HPP

#include <src/car.hpp>
#include <assert.h>
#include <iostream>

const static unsigned int s_road_lanes = 3;
const static unsigned int s_lane_width = 4; // 4 m

unsigned int get_lane(const Car& a) ; 
double get_middle_of_lane(unsigned int lane) ;

#endif /* ROAD_HPP */
