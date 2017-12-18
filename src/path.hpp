#ifndef PATH_HPP
#define PATH_HPP

#include <vector>

struct Path {
    static constexpr const unsigned int s_max_points = 40;
    std::vector<double> m_x;
    std::vector<double> m_y;

    unsigned int m_current_lane;
    double m_current_velocity;
    unsigned int m_target_lane;
    double m_target_velocity;
    double m_max_velocity_possible;

    Path(const std::vector<double>& prev_x, const std::vector<double>& prev_y): m_x(prev_x), m_y(prev_y){}
    Path() {}

    unsigned int const remaining_points();
    unsigned int length() const;
};

#endif /* PATH_HPP */