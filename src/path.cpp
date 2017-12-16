#include <src/path.hpp>
#include <assert.h>
#include <src/common.hpp>
#include <src/planner.hpp>

using namespace std;

unsigned int const Path::remaining_points() {
    assert(s_max_points >= m_x.size());
    return s_max_points - m_x.size();
}

unsigned int Path::length() const {
    return m_x.size();
}

// double const Path::final_velocity() {
//     unsigned int len = m_x.size();

//     if(len < 2) {
//         return 0.0;
//     }
//     double x = m_x[len-1];
//     double y = m_y[len-1];
    
//     double x1 = m_x[len-2];
//     double y1 = m_y[len-2];
//     return distance(x,y,x1,y1) / Planner::s_dt;
// }