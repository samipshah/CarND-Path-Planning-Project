#include <src/planner.hpp>
#include <src/road.hpp>
#include <src/common.hpp>
#include <iostream>
#include <src/spline.h>

using namespace std;

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

Path Planner::_get_trajectory(Planner::CarState a_state, const Car& a_car, const vector<Car>& a_other_cars, const Path& a_prev_path) {
    // logic required for generating trajectories for given state and remaining points.
    Path next_path;
	double ref_vel, ref_theta;
	ref_vel = m_ref_v;
	// vector<double> svals,dvals;
	vector<double> hx,hy; // helper xy
    switch(a_state) {
        case CarState::KEEP_LANE: {
			ref_theta = a_car.m_theta;
			if(a_prev_path.m_x.size() < 2) {
				
				hx.push_back(a_car.m_mapx - cos(ref_theta));
				hy.push_back(a_car.m_mapy - sin(ref_theta));
				
				hx.push_back(a_car.m_mapx);
				hy.push_back(a_car.m_mapy);
			} else {
				unsigned int len = a_prev_path.m_x.size();
				double x = a_prev_path.m_x[len-1];
				double y = a_prev_path.m_y[len-1];
				
				double x1 = a_prev_path.m_x[len-2];
				double y1 = a_prev_path.m_y[len-2];
				ref_theta = atan2(y-y1,x-x1);

				hx.push_back(x1);
				hy.push_back(y1);

				hx.push_back(x);
				hy.push_back(y);
			}

			double ref_s = a_car.m_s;
			unsigned int lane = unsigned(a_car.m_d) / 4;
			double ref_d = 2 + lane*4;
			for(int i=1; i < 4; i++) {
				auto xy = getXY(ref_s+(30*i), ref_d);
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
	cout << hx.size() << endl;
	cout << hx[0] << "," <<  hx[1] <<endl;
	for(int i=0; i<hx.size(); i++) {
		// auto cartesian = getXY(svals[i], dvals[i]);
		
		// shift
		double x = hx[i] - hx[0];
		double y = hy[i] - hy[0];
		// rotate
		double xd = x*cos(ref_theta) - y*sin(ref_theta);
		double yd = x*sin(ref_theta) + y*cos(ref_theta);
		xp.push_back(xd);
		yp.push_back(yd);
	}

	// then beyond what is already computed try to find remaining points from spline
	// convert them back to map coordinate and append
	tk::spline s;
	s.set_points(xp,yp);

	double target_x = 30.0;
	double target_y = s(target_x);
	double target_dist = sqrt((target_x*target_x) + (target_y*target_y));
	double x_add_on = 0.0;
	double N = target_dist/(m_dt*ref_vel);

	int remaining = next_path.remaining_points();
	int len = next_path.m_x.size();
	double ref_x, ref_y;
	if(len > 0) {
		ref_x = next_path.m_x[len-1];
		ref_y = next_path.m_y[len-1];
	} else {
		ref_x = a_car.m_mapx;
		ref_y = a_car.m_mapy;
	}
	for(int i=0; i < remaining; i++) {
		double x_point = x_add_on + (target_x/N);
		double y_point = s(x_point);
		x_add_on = x_point;

		double x_ref = x_point;
		double y_ref = y_point;
		x_point = ref_x + (x_ref*cos(-ref_theta) - y_ref*sin(-ref_theta));
		y_point = ref_y + (x_ref*sin(-ref_theta) + y_ref*cos(-ref_theta));

		next_path.m_x.push_back(x_point);
		next_path.m_y.push_back(y_point);
	}

    return next_path;
}

vector<Planner::CarState> Planner::_possible_transitions(Planner::CarState a_current_state) {
    if(a_current_state == CarState::KEEP_LANE) {
        return {CarState::KEEP_LANE, CarState::PREPARE_LEFT, CarState::PREPARE_RIGHT};
    }

    if(a_current_state == CarState::PREPARE_LEFT) {
        return {CarState::PREPARE_LEFT, CarState::TAKE_LEFT};
    }

    if(a_current_state == CarState::TAKE_LEFT) {
        return {CarState::KEEP_LANE, CarState::TAKE_LEFT};
    }

    if(a_current_state == CarState::PREPARE_RIGHT) {
        return {CarState::PREPARE_RIGHT, CarState::TAKE_RIGHT};
    }

    if(a_current_state == CarState::TAKE_RIGHT) {
        return {CarState::KEEP_LANE, CarState::TAKE_RIGHT};
    }

    return {};
}


vector<Path> Planner::get_trajectories(Planner::CarState a_state, const Car& a_car, const vector<Car>& a_other_cars, const Path& a_prev_path) {
    vector<CarState> l_states = _possible_transitions(a_state);
    vector<Path> l_possible_paths;
    for(Planner::CarState state : l_states) {
        Path a = _get_trajectory(state, a_car, a_other_cars, a_prev_path);
        l_possible_paths.push_back(a);
    }

    return l_possible_paths;
}


int Planner::ClosestWaypoint(double x, double y) {
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

int Planner::NextWaypoint(double x, double y, double theta) {
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
vector<double> Planner::getFrenet(double x, double y, double theta)
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
vector<double> Planner::getXY(double s, double d)
{
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