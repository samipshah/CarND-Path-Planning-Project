#include <src/planner.hpp>
#include <src/road.hpp>
#include <src/common.hpp>
#include <iostream>
#include <src/spline.h>
#include <src/road.hpp>

using namespace std;

struct AM {
	const double x;
	const double y;
	const double theta;
	AM(double x, double y, double theta) : x(x), y(y), theta(theta) {}

	vector<double> convert(vector<double> a);
	vector<double> inverse_convert(vector<double> a);
};

vector<double> AM::convert(vector<double> a) {
	double l_x = a[0] - x;
	double l_y = a[1] - y;

	/// ?????
	return {(l_x*cos(-theta) - l_y*sin(-theta)), (l_x*(sin(-theta))+l_y*cos(-theta))};
}

vector<double> AM::inverse_convert(vector<double> a) {
	double l_x = a[0]; 
	double l_y = a[1];
	/// ???
	return { (l_x*cos(theta) - l_y*sin(theta) + x), (l_x*sin(theta) + l_y*cos(theta) + y)};
}

double vector_mag(double x, double y) {
	return sqrt(x*x + y*y);
}

// vector<double> shift_rotate(vector<double> a, const axis_modification& b) {
// 	// shift
// 	double x = a[0] - b.x;
// 	double y = a[1] - b.y;
// 	// rotate
// 	return { x*cos(b.theta) + y*sin(b.theta), - x*sin(b.theta) + y*cos(b.theta) };
// }

// vector<double> rotate_shift(vector<double> a, const axis_modification& b) {
// 	// rotate
// 	double x = a[0]*cos(b.theta) - a[1]*sin(b.theta);
// 	double y = a[0]*sin(b.theta) + a[1]*cos(b.theta); 
// 	// shift 
// 	return { x + b.x, y + b.y };
// }

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}


// vector<double> _get_lane_velocities(const vector<Car>& a_other_cars) {
// 	vector<double> lane_velocities = vector<double>(3);
// 	vector<int> count = vector<int>(3);
// 	for(Car a : a_other_cars) {
// 		unsigned int lane = get_lane(a);	
// 		lane_velocities[lane] += vector_mag(a.m_vx, a.m_vy);
// 		count[lane] += 1;
// 	}

// 	for(int i=0; i < 3; i++) {
// 		lane_velocities[i] /= count[i];
// 	}
// 	return lane_velocities;
// }

Car get_next_car_in_lane(const Car& a_car, const vector<Car>& a_other_cars, int lane) {
	double s_dist = 10000.0; // really large number
	Car next_car;
	for(Car a : a_other_cars) {
		if(get_lane(a) == lane) {
			double diff = a.m_s - a_car.m_s;
			if(diff > 0 && diff < s_dist) {
				s_dist = diff;
				next_car = a;
			}
		}
	}
	return next_car;
}

double Planner::_get_ref_velocity(double cur, double desired) {
	if(desired > cur) {
		cur += (m_max_acc*.9*s_dt); // 80% of acceleration
	} else {
		cur -= (m_max_acc*.9*s_dt);
	}

	if (cur > s_ref_v) {
		cur = s_ref_v;
	}
	return cur;
}

Path Planner::_get_trajectory(Planner::CarState next_state, const Car& a_car, double max_v, const Path& a_prev_path) {
    // logic required for generating trajectories for given state and remaining points.
    Path next_path;
	double cur_vel;
	double ref_theta = a_car.m_theta;
	vector<double> hx,hy; // helper xy

	// first two points are common in any state
	if(a_prev_path.m_x.size() < 2) {
		hx.push_back(a_car.m_mapx - cos(ref_theta));
		hy.push_back(a_car.m_mapy - sin(ref_theta));
		
		hx.push_back(a_car.m_mapx);
		hy.push_back(a_car.m_mapy);
		cur_vel = vector_mag(a_car.m_vx, a_car.m_vy);
	} else {
		unsigned int len = a_prev_path.m_x.size();
		double x = a_prev_path.m_x[len-1];
		double y = a_prev_path.m_y[len-1];
		
		double x1 = a_prev_path.m_x[len-2];
		double y1 = a_prev_path.m_y[len-2];
		ref_theta = atan2(y-y1,x-x1);
		cur_vel = distance(x,y,x1,y1) / s_dt;

		hx.push_back(x1);
		hy.push_back(y1);

		hx.push_back(x);
		hy.push_back(y);
	}

	// set current and target lane
	double c_lane = get_lane(a_car);
	next_path.m_current_lane = c_lane;
	next_path.m_current_velocity = vector_mag(a_car.m_vx, a_car.m_vy);
	next_path.m_target_lane = c_lane;
    switch(next_state) {
        case CarState::KEEP_LANE: {
			// set reference velocity
			double ref_s = a_car.m_s;
			double ref_d = get_middle_of_lane(get_lane(a_car));
			for(int i=1; i < 4; i++) {
				auto xy = getXY(ref_s+(s_spline_step*i), ref_d);
				hx.push_back(xy[0]);
				hy.push_back(xy[1]);
			}

        } break;
		case CarState::TAKE_LEFT: {
			max_v *= s_dampen_speed; // reduce speed while taking turn
			// current , target lanes
			if(c_lane <= 0) {
				cout << c_lane << ", " << m_lanes << endl;
				return next_path;
			}
			double t_lane = c_lane - 1;
			next_path.m_target_lane = t_lane;

			double ref_s = a_car.m_s;
			double ref_d = get_middle_of_lane(t_lane);
			for(int i=1; i < 4; i++) {
				auto xy = getXY(ref_s+(s_spline_step*i), ref_d);
				hx.push_back(xy[0]);
				hy.push_back(xy[1]);
			}
		} break;
		case CarState::TAKE_RIGHT: {
			max_v *= s_dampen_speed; // reduce speed while taking turn
			// current , target lanes
			if(c_lane >= (m_lanes-1)) {
				cout << c_lane << ", " << m_lanes << endl;
				return next_path;
			}
			double t_lane = c_lane + 1;
			next_path.m_target_lane = t_lane;

			double ref_s = a_car.m_s;
			double ref_d = get_middle_of_lane(t_lane);
			for(int i=1; i < 4; i++) {
				auto xy = getXY(ref_s+(s_spline_step*i), ref_d);
				hx.push_back(xy[0]);
				hy.push_back(xy[1]);
			}
		} break;
        default: {
            return next_path;
        }
    }

	// copy previous points to m_x, m_y
	for(int i = 0; i < a_prev_path.m_x.size(); i++) {
		next_path.m_x.push_back(a_prev_path.m_x[i]);
		next_path.m_y.push_back(a_prev_path.m_y[i]);
	}

	vector<double> xp,yp; // xpoints , ypoints in vehicle coordinates

	// then convert these svals into xy using getXY,convert them to vehicle coordinates
	AM am = AM(hx[1],hy[1],ref_theta);
	for(int i=0; i<hx.size(); i++) {
		// shift
		auto vehicle = am.convert({hx[i], hy[i]});
		xp.push_back(vehicle[0]);
		yp.push_back(vehicle[1]);
	}

	// then beyond what is already computed try to find remaining points from spline
	// convert them back to map coordinate and append
	tk::spline s;
	s.set_points(xp,yp);

	double target_x = 30.0;
	double target_y = s(target_x);
	double target_dist = sqrt((target_x*target_x) + (target_y*target_y));
	double x_add_on = 0.0;

	int remaining = next_path.remaining_points();
	int len = next_path.m_x.size();
	double ref_x, ref_y;

	for(int i=0; i < remaining; i++) {
		cur_vel = _get_ref_velocity(cur_vel, max_v);
		next_path.m_current_velocity = cur_vel;
		assert(cur_vel > 0);
		assert(cur_vel <= s_ref_v);
		double N = target_dist/(s_dt*cur_vel);
		double x_point = x_add_on + (target_x/N);
		double y_point = s(x_point);
		x_add_on = x_point;

		auto map = am.inverse_convert({x_point, y_point});
		next_path.m_x.push_back(map[0]);
		next_path.m_y.push_back(map[1]);
	}

    return next_path;
}

vector<Planner::CarState> Planner::_possible_transitions(Planner::CarState a_current_state, const Path& prev_path) {
    if(a_current_state == CarState::KEEP_LANE) {
        return {CarState::KEEP_LANE, CarState::TAKE_LEFT, CarState::TAKE_RIGHT};
    }

    if(a_current_state == CarState::TAKE_LEFT) {
		// if(prev_path.m_current_lane == prev_path.m_target_lane + 1) {
		// 	return {CarState::TAKE_LEFT};
		// }
        return {CarState::KEEP_LANE, CarState::TAKE_LEFT};
    }

    if(a_current_state == CarState::TAKE_RIGHT) {
		// if(prev_path.m_current_lane == prev_path.m_target_lane - 1) {
		// 	return {CarState::TAKE_RIGHT};
		// }
        return {CarState::KEEP_LANE, CarState::TAKE_RIGHT};
    }

    return {};
}

double _get_next_car_velocity(const Car& a_car, const vector<Car>& a_other_cars, unsigned int lane) {
	double max_v = Planner::s_ref_v;
	Car next_car = get_next_car_in_lane(a_car, a_other_cars, lane);
	if(fabs(next_car.m_id - (-2)) > std::numeric_limits<double>::epsilon()) {
		if (fabs(a_car.m_s - next_car.m_s) < 50.0) {
			max_v = vector_mag(next_car.m_vx, next_car.m_vy);
		}
	}
	return max_v;
}

vector<std::pair<Planner::CarState,Path>> Planner::get_trajectories(Planner::CarState a_state, const Car& a_car, const vector<Car>& a_other_cars, const Path& a_prev_path) {
    vector<CarState> l_states = _possible_transitions(a_state, a_prev_path);
    vector<std::pair<CarState,Path>> l_possible_paths;
	
	double max_v = _get_next_car_velocity(a_car, a_other_cars, get_lane(a_car));
	if(a_state == CarState::KEEP_LANE && max_v >= s_ref_v) {
        Path a = _get_trajectory(CarState::KEEP_LANE, a_car, max_v, a_prev_path);
		double t_vel = _get_next_car_velocity(a_car, a_other_cars, a.m_target_lane);
		a.m_target_velocity = (t_vel < Planner::s_ref_v) ? t_vel : Planner::s_ref_v;
        l_possible_paths.push_back(std::pair<CarState,Path>(CarState::KEEP_LANE, a));
	} else {
		for(Planner::CarState state : l_states) {
			Path a = _get_trajectory(state, a_car, max_v, a_prev_path);
			double t_vel = _get_next_car_velocity(a_car, a_other_cars, a.m_target_lane);
			a.m_target_velocity = (t_vel < Planner::s_ref_v) ? t_vel : Planner::s_ref_v;
			l_possible_paths.push_back(std::pair<CarState,Path>(state, a));
		}
	}
    return l_possible_paths;
}


int Planner::ClosestWaypoint(double x, double y) const {
	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < m_maps_x.size(); i++) {
		double map_x = m_maps_x[i];
		double map_y = m_maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen) {
			closestLen = dist;
			closestWaypoint = i;
		}
	}

	return closestWaypoint;
}

int Planner::NextWaypoint(double x, double y, double theta) const {
	int closestWaypoint = ClosestWaypoint(x,y);

	double map_x = m_maps_x[closestWaypoint];
	double map_y = m_maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  	angle = min(2*pi() - angle, angle);

  	if(angle > pi()/4) {
		closestWaypoint++;
		if (closestWaypoint == m_maps_x.size()) {
			closestWaypoint = 0;
		}
	}

  	return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> Planner::getFrenet(double x, double y, double theta) const
{
	int next_wp = NextWaypoint(x,y,theta);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = m_maps_x.size()-1;
	}

	double n_x = m_maps_x[next_wp]-m_maps_x[prev_wp];
	double n_y = m_maps_y[next_wp]-m_maps_y[prev_wp];
	double x_x = x - m_maps_x[prev_wp];
	double x_y = y - m_maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-m_maps_x[prev_wp];
	double center_y = 2000-m_maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(m_maps_x[i],m_maps_y[i],m_maps_x[i+1],m_maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> Planner::getXY(double s, double d) const {
	int prev_wp = -1;

	while(s > m_maps_s[prev_wp+1] && (prev_wp < (int)(m_maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%m_maps_x.size();

	double heading = atan2((m_maps_y[wp2]-m_maps_y[prev_wp]),(m_maps_x[wp2]-m_maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-m_maps_s[prev_wp]);

	double seg_x = m_maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = m_maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

// spline 