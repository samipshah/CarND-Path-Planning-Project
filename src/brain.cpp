#include <vector>
#include <iostream>
#include <src/brain.hpp>
#include <src/road.hpp>


using namespace std;
// try changing lane when speed is less than 50 , if in a lane and speed of 50
// is achieved just keep the lane
// also other main goals

// states - KEEP_LANE, PREPARE_LEFT, PREPARE_RIGHT, TAKE_LEFT, TAKE_RIGHT

constexpr static const double s_weights[] = { 
    2,
    5,
};

double Brain::_cost(std::pair<Planner::CarState,Path>& pair, const vector<Car>& other_cars, const Planner& p) {
    // very high cost where collision occurs
    // if the car is path is way too close to any of the other cars predicted locations
    // associate very high cost
    cout << "Cost " << unsigned(pair.first) << ":" ;
    double cost = 0.0;
    // incomplete trajectories
    Path& a = pair.second;
    if(a.remaining_points() > 0) {
        cost += 10000.0; // large number for incomplete trajectories
    }

    // for lane changing trajectories calculate if it is collision safe
    if(a.m_current_lane != a.m_target_lane) {
        double x = a.m_x[Path::s_max_points-1];
        double y = a.m_y[Path::s_max_points-1];
        double x1 = a.m_x[Path::s_max_points-2];
        double y1 = a.m_y[Path::s_max_points-2];
        double theta = atan2(y1-y,x1-x);
        auto last = p.getFrenet(x, y, theta);

        // collision possibilities 
        for (Car c : other_cars) {
            double diff = c.m_s - last[0];
            if(get_lane(c) == a.m_current_lane) {
                if(diff < 0.0) {
                    // forget about cars behind us in the same lane
                    continue;
                }

                // for cars in front of us
                if(diff < a.m_current_velocity*Planner::s_dt*(Path::s_max_points*.7)) {
                    cost += 10000.0;
                    cout << "front car too close " << cost << ",";
                    break;
                }
            }

            if(get_lane(c) == a.m_target_lane) {
                if (diff < 0.0) {
                    // for cars behind us make sure it is safe to take turn
                    if(c.current_speed() < a.m_target_velocity) {
                        if(abs(diff) < c.current_speed()*Planner::s_dt*(Path::s_max_points*.2)) {
                            cost += 10000.0;
                            cout << "cars behind in target lane too close" << cost << ",";
                            break;
                        }
                        continue;
                    } else {
                        if(abs(diff) < c.current_speed()*Planner::s_dt*(Path::s_max_points*2)) {
                            cost += 10000.0;
                            cout << "cars behind in target too fast " << cost << ",";
                            break;
                        } 
                        continue;
                    }
                }

                // faster car in front of us in target len 
                if(c.current_speed() > a.m_target_velocity) {
                    if(abs(diff) < a.m_target_velocity*Planner::s_dt*(Path::s_max_points*.5)) {
                        cost += 10000.0;
                        cout << "Not yet safe to change " << cost << ",";
                        break;
                    } 
                    continue;
                }

                // slower car in front of us in target lane 
                if (diff < a.m_target_velocity*Planner::s_dt*(Path::s_max_points*2)) {
                    cost += 10000.0;
                    cout << "front car in target too close " << cost << ",";
                    break;
                }
            }

            // forget about the cars in the other lanes
        }
    }

    if (cost > 10000.0) {
        return cost;
    }

    // cost proportional to difference between max speed and lane speed
    cost += (Planner::s_ref_v - a.m_target_velocity)*s_weights[0];
    cout << cost << ",";

    // lane change should have a cost , cast to double is done to avoid ambiguous function call
    cost += (abs(double(double(a.m_target_lane) - double(a.m_current_lane))))*s_weights[1];
    cout << cost << ",";

    // heavy penalty for making a different decision when taking turn
    if(m_current_state == Planner::CarState::TAKE_LEFT && pair.first != Planner::CarState::TAKE_LEFT) {
		if(m_current_lane == m_target_lane + 1) {
			cost += 10000.0;
		}
    }

    if(m_current_state == Planner::CarState::TAKE_RIGHT && pair.first != Planner::CarState::TAKE_RIGHT) {
		if(m_current_lane == m_target_lane - 1) {
			cost += 10000.0;
		}
    }

    cout << endl;
    return cost;
}

Path Brain::next_path(Car& a_car, vector<Car>& other_cars, Planner& a_planner, const Path& prev_path) {
    // predict next locations of other cars 
    unsigned int calculated = prev_path.length();
    for(Car& a : other_cars) {
        a.update_location(calculated*0.02); // 0.02 is the timestep
    }
    Path l_p = prev_path;
    if(m_target_lane != 5) {
        l_p.m_target_lane = m_target_lane;
    }

    // possible transitions
    vector<std::pair<Planner::CarState,Path>> l_possible_paths = a_planner.get_trajectories(m_current_state, a_car, other_cars, l_p);
    unsigned int new_points = Path::s_max_points - calculated;
    for(Car& a : other_cars) {
        a.update_location(new_points*0.02);
    }

    // predict other cars position based on gaussian distribution around its location
    double min_cost = 100000.0;
    pair<Planner::CarState,Path> preferred;
    // Path preferred_path = l_p;
    // Planner::CarState preferred_state;
    cout << "----" << endl;
    for (auto pair : l_possible_paths) {
        Path a = pair.second;
        double cost = _cost(pair, other_cars, a_planner);
        cout << a.m_x.size() << "," << cost << endl;
        if(min_cost > cost) {
            min_cost = cost;
            preferred = pair;
            // preferred_path = a;
            // preferred_state = pair.first;
        }
    }

    // assign it for the next state
    m_current_state = preferred.first;
    m_current_lane = preferred.second.m_current_lane;
    m_target_lane = preferred.second.m_target_lane;

    cout << "Preferred : " << unsigned(m_current_state) << endl;

    return preferred.second;
}
