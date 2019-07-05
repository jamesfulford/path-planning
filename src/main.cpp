#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
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
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          int prev_size = previous_path_x.size();

          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          double target_speed = 49.5 / 2.24; // 49.5 MPH -> meters/sec
          int target_lane = 1; // Starts in middle lane
          // [0, 1, 2]

          // Define anchor points
          vector<double> pts_x;
          vector<double> pts_y;
          
          double ref_x;
          double ref_y;
          double ref_yaw;
          if (prev_size < 2) {
            std::cout << "prev_size is low" << std::endl;
            pts_x.push_back(car_x - cos(car_yaw));
            pts_y.push_back(car_y - sin(car_yaw));

            pts_x.push_back(car_x);
            pts_y.push_back(car_y);
            
            ref_x = car_x;
            ref_y = car_y;
            ref_yaw = car_yaw;
          } else {
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];
            
            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
            
            pts_x.push_back(ref_x_prev);
            pts_y.push_back(ref_y_prev);

            pts_x.push_back(ref_x);
            pts_y.push_back(ref_y);
          }
          
          
          vector<double> next_waypoint;
          for (int i = 1; i < 4; i++) {
            next_waypoint = getXY(
              car_s + (i * 30),
              2 + (4 * target_lane),
              map_waypoints_s, map_waypoints_x, map_waypoints_y
            );
            pts_x.push_back(next_waypoint[0]);
            pts_y.push_back(next_waypoint[1]);
          }
          std::cout << "inferred waypoints: " << pts_x.size() << std::endl;
          
          // Rotate pts to ref's perspective
          for (int i = 0; i < pts_x.size(); i++) {
            double shift_x = pts_x[i] - ref_x;
            double shift_y = pts_y[i] - ref_y;
            pts_x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
            pts_y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
          }
          std::cout << "shifted to local coords" << std::endl;
          
          tk::spline path;
          
          path.set_points(pts_x, pts_y);
          
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for (int i = 0; i < prev_size; i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          std::cout << "prev_size is " << prev_size << std::endl;
          
          double target_x = 30.0;
          double target_y = path(target_x);
          double target_d = distance(0, 0, target_x, target_y);
          // How many chunks needed to cover distance given target speed and chunk time is 0.02 seconds?
          double N = target_d / (0.02 * target_speed);

          // Can't make it all the way, but squeeze in as many points as you can
          for (int i = 0; i < 50 - prev_size; i++) {
            double x = (target_x / N) * (i + 1);
            double y = path(x);
            next_x_vals.push_back(
              (x * cos(ref_yaw) - y * sin(ref_yaw)) + ref_x
            );
            next_y_vals.push_back(
              (x * sin(ref_yaw) + y * cos(ref_yaw)) + ref_y
            );
          }
          
//           vector<double> next_s_vals;
//           vector<double> next_d_vals;
//           for (int i = 0; i < 50; i++) {
//             next_s_vals.push_back(car_s + (i * 0.5));
//             next_d_vals.push_back(car_d);
//           }
          
//           for (int i = 0; i < next_s_vals.size(); i++) {
//             vector<double> xy = getXY(next_s_vals[i], next_d_vals[i], map_waypoints_s, map_waypoints_x, map_waypoints_y);
//             next_x_vals.push_back(xy[0]);
//             next_y_vals.push_back(xy[1]);
//           }


          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";
          for(int i = 0; i < next_x_vals.size(); i++) {
            std::cout << i << ": (" << next_x_vals[i] << ", " << next_y_vals[i] << ")" << std::endl;
          }
          std::cout << "Sending response with points.size() " << next_x_vals.size() <<  std::endl;

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