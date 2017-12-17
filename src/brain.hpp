#ifndef BRAIN_HPP
#define BRAIN_HPP

#include <vector>
#include <src/path.hpp>
#include <src/planner.hpp>
#include <src/car.hpp>

struct Brain {
public:
    double m_goal_speed  = 50; // mph
    Planner::CarState m_current_state = Planner::CarState::KEEP_LANE;
    unsigned int m_current_lane;
    unsigned int m_target_lane = 5;
    Path next_path(Car& car, std::vector<Car>& other_cars, Planner& planner, const Path& prev_path);
private:
    double _cost(std::pair<Planner::CarState, Path>& a, const std::vector<Car>& other_cars, const Planner& p);
};

#endif /* BRAIN_HPP */