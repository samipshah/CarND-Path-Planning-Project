#include <src/road.hpp>


unsigned int get_lane(const Car& a) { 
    if(a.m_d < 0 || a.m_d >= 12) { 
        std::cout << "What? : " << a.m_d << std::endl;
        return 0; 
    }; 
    return a.m_d/4; 
}

double get_middle_of_lane(unsigned int lane) { return 2 + lane*s_lane_width;}