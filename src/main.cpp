#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "path_builder.h"
#include "car.h"
#include "predictor.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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
  PathBuilder path_builder{ map_waypoints_s, map_waypoints_x, map_waypoints_y };
  int target_lane = 1;
  double ref_vel = 0;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &path_builder, &target_lane, &ref_vel]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          json &data_json = j[1];
          
          struct EgoCar ego_car;
          ego_car.x = data_json["x"];
          ego_car.y = data_json["y"];
          ego_car.s = data_json["s"];
          ego_car.d = data_json["d"];
          ego_car.yaw = data_json["yaw"];
          ego_car.speed = data_json["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = data_json["previous_path_x"];
          auto previous_path_y = data_json["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = data_json["end_path_s"];
          double end_path_d = data_json["end_path_d"];

          Predictor predictor { target_lane, ego_car, data_json["sensor_fusion"], (int) previous_path_x.size() };
          Prediction prediction = predictor.get_prediction();

          vector<LaneAndSpeed> lanes_and_speeds = path_builder.get_possible_lanes_and_speeds(prediction, target_lane, ref_vel);

          LaneAndSpeed best_lane_and_speed = path_builder.get_best_lane_and_speed(lanes_and_speeds, target_lane, ref_vel);
          target_lane = best_lane_and_speed.lane;
          ref_vel = best_lane_and_speed.speed;

          vector<vector<double>> next_points = path_builder.build_path(
            previous_path_x, previous_path_y, ego_car, target_lane, ref_vel
          );

          json msgJson;
          msgJson["next_x"] = next_points[0];
          msgJson["next_y"] = next_points[1];

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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