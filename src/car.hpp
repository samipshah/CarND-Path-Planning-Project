#ifndef CAR_HPP
#define CAR_HPP

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
};

#endif /* CAR_HPP */