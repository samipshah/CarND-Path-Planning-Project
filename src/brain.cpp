#include <vector>
#include <iostream>
#include <src/brain.hpp>
#include <src/road.hpp>


using namespace std;
// try changing lane when speed is less than 50 , if in a lane and speed of 50
// is achieved just keep the lane
// also other main goals

// states - KEEP_LANE, PREPARE_LEFT, PREPARE_RIGHT, TAKE_LEFT, TAKE_RIGHT



double Brain::_cost(Path& a, const vector<Car>& other_cars, const Planner& p) {
    // very high cost where collision occurs
    // if the car is path is way too close to any of the other cars predicted locations
    // associate very high cost
    cout << "Cost:";
    double cost = 0.0;
    // incomplete trajectories
    if(a.remaining_points() > 0) {
        return 10000.0; // large number for incomplete trajectories
    }

    // for lane changing trajectories calculate if it is collision safe
    if(a.m_current_lane != a.m_target_lane) {
        double x = a.m_x[Path::s_max_points-1];
        double y = a.m_y[Path::s_max_points-1];
        double x1 = a.m_x[Path::s_max_points-2];
        double y1 = a.m_y[Path::s_max_points-2];
        double theta = atan2(y1-y,x1-x);
        auto last = p.getFrenet(x, y, theta);
        double clear_distance = a.m_current_velocity*0.02*50;
        // collision possibilities 
        for (Car c : other_cars) {
            double diff = c.m_s - last[0];
            
            if(get_lane(c) == a.m_current_lane) {
                if(diff < 0.0) {
                    // forget about cars behind us in the same lane
                    continue;
                }

                if(diff < a.m_current_velocity*Planner::s_dt*Path::s_max_points) {
                    return 10000.0;
                }

            }

            if(get_lane(c) == a.m_target_lane) {
                if (diff < 0.0) {
                    if(abs(diff) < c.current_speed()*Planner::s_dt*Path::s_max_points) {
                        return 10000.0;
                    } else {
                        continue;
                    }
                }

                // cars in front of us in target len
                if (diff < a.m_target_velocity*Planner::s_dt*Path::s_max_points) {
                    return 10000.0;
                }
            }
            // forget about the cars in the other lanes
        }
    }

    // cost proportional to difference between max speed and lane speed
    cost += (Planner::s_ref_v - a.m_current_velocity)*10; 
    cout << cost << ","; 
    cost += (Planner::s_ref_v - a.m_target_velocity)*10;
    cout << cost << ",";

    // lane change should have a cost , cast to double is done to avoid ambiguous function call
    cost += (abs(double(double(a.m_target_lane) - double(a.m_current_lane))))*10;
    cout << cost << ",";

    // cost for being too close to a car
    
    cout << endl;
    return cost;
}

Path Brain::next_path(Car& a_car, vector<Car>& other_cars, Planner& a_planner, const Path& prev_path) {
    // predict next locations of other cars 
    unsigned int calculated = prev_path.length();
    for(Car& a : other_cars) {
        a.update_location(calculated*0.02); // 0.02 is the timestep
    }

    // possible transitions
    vector<std::pair<Planner::CarState,Path>> l_possible_paths = a_planner.get_trajectories(m_current_state, a_car, other_cars, prev_path);
    unsigned int new_points = Path::s_max_points - calculated;
    for(Car& a : other_cars) {
        a.update_location(new_points*0.02);
    }

    // predict other cars position based on gaussian distribution around its location
    double min_cost = 10000.0;
    Path preferred_path = prev_path;
    cout << "----" << endl;
    for (auto pair : l_possible_paths) {
        Path a = pair.second;
        double cost = _cost(a, other_cars, a_planner);
        cout << a.m_x.size() << "," << cost << endl;
        if(min_cost > cost) {
            min_cost = cost;
            preferred_path = a;
            m_current_state = pair.first;
        }
    }

    return preferred_path;
}
