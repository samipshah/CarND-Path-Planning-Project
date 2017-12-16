#ifndef CAR_HPP
#define CAR_HPP
#include <cmath>

struct Car {
    // int m_right_max;
    // int m_left_max;
    // int m_throttle_max;
    // int m_throttle_min;
    double m_id;
    double m_mapx;
    double m_mapy;
    double m_vx;
    double m_vy;
    double m_s;
    double m_d;
    double m_theta;

    /* Constructor for other cars */
    Car(double a_id, double a_mapx, double a_mapy, double a_vx, double a_vy, double a_s, double a_d) 
    : m_id(a_id),
    m_mapx(a_mapx),
    m_mapy(a_mapy),
    m_vx(a_vx),
    m_vy(a_vy),
    m_s(a_s),
    m_d(a_d){}

    /* Constructor for current cars */
    Car(double a_id, double a_x, double a_y, double a_yaw) : m_id(a_id), m_mapx(a_x), m_mapy(a_y),m_theta(a_yaw) {}

    Car() : m_id(-2) {
    }

    double current_speed() {
        return sqrt((m_vx*m_vx) + (m_vy*m_vy));
    };

    bool same_lane(const Car& b);

    void update_location(double timestep) {
        // m_mapx += m_vx*timestep;
        // m_mapy += m_vy*timestep;
        m_s += current_speed()*timestep;
    }

};

#endif /* CAR_HPP */