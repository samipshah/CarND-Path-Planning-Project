#ifndef PLANNER_HPP
#define PLANNER_HPP

#include <src/path.hpp>
#include <src/car.hpp>

Car get_next_car_in_lane(const Car& a_car, const std::vector<Car>& a_other_cars, int lane);
double get_next_car_velocity(const Car& a_car, const std::vector<Car>& a_other_cars, unsigned int lane);
std::vector<Car> get_nearby_cars_in_lanes(const Car& a_car, const std::vector<Car>& a_other_cars, int current_lane, int target_lane);
std::vector<Car> get_cars_lane(const std::vector<Car>& a_other_cars, const Car& a_car);

struct Planner {
    const int m_lanes = 3;
    const double m_prediction_time = 1; // 1 seconds
    const unsigned int m_num_points = 5; // calculate just 5 points on the trajectory use spline later to extrapolate 50
    static constexpr const double s_ref_v = 22; // 50 mph = 50/2.24 = 22.32 m/s
    const double m_max_acc = 10;  // m/s2
    const double m_max_jerk = 10; // m/s3 
    static constexpr const double s_dt = 0.02; // time step in seconds
    static constexpr const double s_spline_step = 40; // 30 m step
    static constexpr const double s_dampen_speed = .9; // dampen while taking turn
    

    std::vector<double> m_maps_x;
    std::vector<double> m_maps_y;
    std::vector<double> m_maps_s;
    std::vector<double> m_maps_dx;
    std::vector<double> m_maps_dy;

    enum class CarState : unsigned int { KEEP_LANE , PREPARE_LEFT, PREPARE_RIGHT, TAKE_LEFT, TAKE_RIGHT };
    // give sension fusion, car
    std::vector<std::pair<CarState,Path>> get_trajectories(CarState state, const Car& car, const std::vector<Car>& other_cars, const Path& prev_path);
    Planner(std::vector<double> a, std::vector<double> b, std::vector<double> c, std::vector<double> d, std::vector<double> e):
    m_maps_x(a),
    m_maps_y(b),
    m_maps_s(c),
    m_maps_dx(d),
    m_maps_dy(e) {}
    int ClosestWaypoint(double x, double y) const;
    int NextWaypoint(double x, double y, double theta) const;
    std::vector<double> getFrenet(double x, double y, double theta) const;
    std::vector<double> getXY(double s, double d) const;

private:
    Path _get_trajectory(CarState state, const Car& car, const Path& prev_path, const std::vector<Car>& a_other_cars);
    std::vector<Planner::CarState> _possible_transitions(Planner::CarState current_state, const Path& prev_path);
    double _get_ref_velocity(double cur, double desired, double t_n);
};

#endif /* PLANNER_HPP */