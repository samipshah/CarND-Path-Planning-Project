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
    Path next_path(Car& car, std::vector<Car>& other_cars, Planner& planner, const Path& prev_path);
private:
    double _cost(Path& a, const std::vector<Car>& other_cars);
};

#endif /* BRAIN_HPP */