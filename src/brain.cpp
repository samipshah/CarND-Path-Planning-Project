#include <vector>
#include <iostream>
#include "brain.hpp"

using namespace std;
// try changing lane when speed is less than 50 , if in a lane and speed of 50
// is achieved just keep the lane
// also other main goals

// states - KEEP_LANE, PREPARE_LEFT, PREPARE_RIGHT, TAKE_LEFT, TAKE_RIGHT



double Brain::_cost(Path& a, const vector<Car>& other_cars) {
    if(a.remaining_points() != a.m_max_points) {
        return 0.0;
    }
    return 100.0;
}

Path Brain::next_path(Car& a_car, vector<Car>& other_cars, Planner& a_planner, const Path& prev_path) {
    // possible transitions
    vector<Path> l_possible_paths = a_planner.get_trajectories(m_current_state, a_car, other_cars, prev_path);

    // predict other cars position based on gaussian distribution around its location
    double min_cost = 10000.0;
    Path preferred_path = prev_path;
    cout << "----" << endl;
    for (Path a : l_possible_paths) {
        double cost = _cost(a, other_cars);
        cout << a.m_x.size() << "," << cost << endl;
        if(min_cost > cost) {
            min_cost = cost;
            preferred_path = a;
        }
    }

    return preferred_path;
}
