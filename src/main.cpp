#include <fstream>
#include <src/common.hpp>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include <src/brain.hpp>
#include <src/path.hpp>
#include <src/road.hpp>

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

	Brain r_brain;
	Planner r_planner(map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy);
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&r_brain,&r_planner](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

						double theta = deg2rad(car_yaw);
						double car_vx = car_speed*cos(theta);
						double car_vy = car_speed*sin(theta);
						Car l_car(-1, car_x, car_y, car_vx, car_vy, car_s, car_d);
						l_car.m_theta = theta;

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"].get<vector<double> >();
          	auto previous_path_y = j[1]["previous_path_y"].get<vector<double> >();
						int previous_size = previous_path_x.size();
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

						vector<Car> other_cars;
						for(int i =0; i < sensor_fusion.size(); i++) {
							if(sensor_fusion[i][6] < 0 || sensor_fusion[i][6] > 12) {
								continue;
							}
							other_cars.emplace_back(sensor_fusion[i][0], sensor_fusion[i][1], sensor_fusion[i][2], sensor_fusion[i][3], sensor_fusion[i][4], sensor_fusion[i][5], sensor_fusion[i][6]);
						}

						// vector<double> previous_path_s;
						// vector<double> previous_path_d;

						// for(int i=0; i<previous_size; i++) {
						// 	vector<double> frenet = r_planner.getFrenet(previous_path_x[i], previous_path_y[i], theta);
						// 	previous_path_s.push_back(frenet[0]);
						// 	previous_path_d.push_back(frenet[1]);
						// }
						
						// for(int i=0; i < previous_path_s.size(); i++) {
						// 	cout << "s:" << previous_path_s[i] << ",d:" << previous_path_d[i] << endl;
						// }
						// cout << "cars:" << car_s << ",card:" << car_d << ",carx:" << car_x << ",cary:" << car_y << endl;
						Path l_prev_path(previous_path_x, previous_path_y);
						l_prev_path.m_current_lane = get_lane(l_car);
						Car temp;
						temp.m_d = end_path_d;
						l_prev_path.m_target_lane = get_lane(temp);
						// get current car object , and sensor fusion data , trajectory planner to
						// brain finds out possible state transitions,
						// using trajectory planner to find all trajectories
						// using cost functions to decide on the lowest cost function
						// spews out the trajectory with lowest cost function
						// cost functions include avoid collision , avoid breaking laws, optimize on speed
						Path path = r_brain.next_path(l_car, other_cars, r_planner, l_prev_path);
						// cout << "size:" << path.m_path_s.size() << endl;
						// for(int i=0; i < path.m_path_s.size(); i++) {
						// 	vector<double> xy = r_planner.getXY(path.m_path_s[i], path.m_path_d[i]);
						// 	next_x_vals.push_back(xy[0]);
						// 	next_y_vals.push_back(xy[1]);
						// }

						// for(int i=0; i < path.m_x.size(); i++) {
						// 	cout << "x:" << path.m_x[i] << ",y:" << path.m_y[i] << endl;
						// }

						// use spline here

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	msgJson["next_x"] = path.m_x;
          	msgJson["next_y"] = path.m_y;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
