#ifndef PATH_HPP
#define PATH_HPP

#include <vector>

struct Path {
    unsigned int m_max_points = 50;
    // std::vector<double> m_prev_x;
    // std::vector<double> m_prev_y;

    std::vector<double> m_x;
    std::vector<double> m_y;

    Path(const std::vector<double>& prev_x, const std::vector<double>& prev_y): m_x(prev_x), m_y(prev_y){}
    Path(const Path& a) {
        // m_prev_x = a.m_prev_x;
        // m_prev_y = a.m_prev_y;
        m_x = a.m_x;
        m_y = a.m_y;
    }
    Path() {}

    unsigned int remaining_points();
    unsigned int length();
};

#endif /* PATH_HPP */