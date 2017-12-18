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
    10,
    20,
    5,
    20
};

double Brain::_cost(std::pair<Planner::CarState,Path>& pair, const vector<Car>& other_cars, const Planner& p, const Car& a_car) {

    // very high cost where collision occurs
    // if the car is path is way too close to any of the other cars predicted locations
    // associate very high cost
    cout << "Cost " << unsigned(pair.first) << ":" ;
    double cost = 0.0;
    // incomplete trajectories
    Path& a = pair.second;
    if(a.remaining_points() > 0) {
        cost += 10000.0; // large number for incomplete trajectories
        cout << "incomplete trajectory " << cost << ",";
    }

    // cost inversely proportional to distance
    // for lane changing trajectories calculate if it is collision safe
    if(a.m_current_lane != a.m_target_lane) {
        double x = a.m_x[Path::s_max_points-1];
        double y = a.m_y[Path::s_max_points-1];
        double x1 = a.m_x[Path::s_max_points-2];
        double y1 = a.m_y[Path::s_max_points-2];
        double theta = atan2(y1-y,x1-x);
        auto last = p.getFrenet(x, y, theta);
        Car temp;
        temp.m_id = -1;
        temp.m_s = last[0];
        temp.m_d = last[1];
        temp.m_mapx = x;
        temp.m_mapy = y;
        vector<Car> cars = get_nearby_cars_in_lanes(temp, other_cars, a.m_current_lane, a.m_target_lane);
        // car in front
        if(cars[0].initialized())  {
            double diff = cars[0].m_s - temp.m_s;
            cost += 5000/(diff*diff);
            cout << "front car (" << diff << "," << a.m_current_lane << "," << a.m_target_lane << ") " << cost << ",";
        }

        // car in front target
        if(cars[1].initialized()) {
            double diff = cars[1].m_s - temp.m_s;
            cost += 5000/(diff*diff);
            cout << "front target car (" << diff << "," << a.m_current_lane << "," << a.m_target_lane << ") " << cost << ",";
        }

        // car in back target
        if(cars[2].initialized()) {
            double diff = cars[2].m_s - temp.m_s;
            cost += 5000/(diff*diff);
            cout << "back target car (" << diff << "," << a.m_current_lane << "," << a.m_target_lane << ") " << cost << ",";
            if(fabs(diff) < 100 && cars[2].current_speed() > a.m_target_velocity) {
                cost += (cars[2].current_speed() - a.m_target_velocity)*s_weights[3];
                cout << "back target car speed " << cost;
            }
        }
    }

    // cost related to distance to the lane which is fastest probably
    vector<Car> next_cars = get_cars_lane(other_cars, a_car);
    Car fastest;
    for(Car a : next_cars) {
        if(a.current_speed() > fastest.current_speed()) {
            fastest = a;
        }
    }	

    if(fastest.initialized()) {
        cost += abs((double)get_lane(fastest) - (double)a.m_target_lane)*s_weights[2];
        cout << "not going to fastest lane: (" << get_lane(fastest) << "," << a.m_target_lane << ") " << cost << ",";
    }

    // cost proportional to difference between max speed and lane speed
	double t_vel = get_next_car_velocity(a_car, other_cars, a.m_target_lane);
    cout << "(" << t_vel << ")" << ",";
	double m_max_velocity_possible = (t_vel < Planner::s_ref_v) ? t_vel : Planner::s_ref_v;
    cost += (Planner::s_ref_v - m_max_velocity_possible)*s_weights[0];
    cout << "speed " << cost << ",";

    // lane change should have a cost , cast to double is done to avoid ambiguous function call
    cost += (abs(double(double(a.m_target_lane) - double(a.m_current_lane))))*s_weights[1];
    cout << "lane change " << cost << ",";

    // heavy penalty for making a different decision when taking turn
    if(m_current_state == Planner::CarState::TAKE_LEFT && pair.first != Planner::CarState::TAKE_LEFT) {
        // path has not yet achieved target lane
		if(pair.second.m_current_lane == m_target_lane + 1) {
			cost += 10000.0;
            cout << "path not yet in target lane " << cost << ",";
		}
    }

    if(m_current_state == Planner::CarState::TAKE_RIGHT && pair.first != Planner::CarState::TAKE_RIGHT) {
        // path has not yet achieved target lane
		if(a.m_current_lane == m_target_lane - 1) {
			cost += 10000.0;
            cout << "path not yet in target lane " << cost << ",";
		}
    }

    // cost for not having enough state transitions
    bool notenough = false;
    for(int i = 0; i < 5; i++) {
        if(m_previous_state[0] != pair.first) {
            notenough = true;
        } 
    }

    if(notenough) {
        // add cost to it
        // cost += 20.0;
        cout << "not enough yet cost " << cost << ",";
    }

    cout << endl;
    return cost;
}

Path Brain::next_path(Car& a_car, vector<Car>& other_cars, Planner& a_planner, const Path& prev_path) {
    cout << "----" << endl;
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
    for (auto pair : l_possible_paths) {
        Path a = pair.second;
        double cost = _cost(pair, other_cars, a_planner, a_car);
        cout << a.m_x.size() << "," << cost << endl;
        if(min_cost > cost) {
            min_cost = cost;
            preferred = pair;
        }
    }

    m_count += 1;
    m_previous_state[m_count%5] = preferred.first;
    m_current_state = preferred.first;
    m_current_lane = preferred.second.m_current_lane;
    m_target_lane = preferred.second.m_target_lane;

    cout << "Preferred : " << unsigned(preferred.first) << endl;
    cout << "----" << endl;
    return preferred.second;
}
