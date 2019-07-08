#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

enum LaneState {
  STAY = 0,
  WAIT = 1
};

enum SpeedState {
  FULL = 0,
  MATCH = 1
};

double get_sensor_fusion_speed(vector<double> sensor_fusion) {
  return distance(0, 0, sensor_fusion[3], sensor_fusion[4]);
}

double get_sensor_fusion_future_position(vector<double> sensor_fusion, int prev_size) {
  return sensor_fusion[5] + (double) prev_size * 0.02 * get_sensor_fusion_speed(sensor_fusion);
}

int get_sensor_fusion_lane(vector<double> sensor_fusion) {
  return std::round((sensor_fusion[6] - 2) / 4.0);
}

double min(double a, double b) {
  if (a < b) {
    return a;
  }
  return b;
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
  
  int lane = 1;
  double velocity_mph = 0.0;
  LaneState lane_state = LaneState::STAY;
  SpeedState speed_state = SpeedState::FULL;

  h.onMessage([&velocity_mph,&lane,&lane_state,&speed_state,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
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
          
          //
          // CONTROL
          //
          
          // adjust target speed and lanes based on sensor fusion data
          double current_s = car_s;
          if (prev_size > 0) {
            // Predict where we will be in the future
            current_s = end_path_s;
          }

          double max_acceleration_mph = 0.8;
          double max_deceleration_mph = -0.8;
          double speed_adjustment_mph = max_acceleration_mph;
          
          // Find obstacles
          for (int i = 0; i < sensor_fusion.size(); i++) {
            std::cout << "sensor_fusion_lane: " << get_sensor_fusion_lane(sensor_fusion[i]) << std::endl;
            if (  // In my lane
              get_sensor_fusion_lane(sensor_fusion[i]) == lane
            ) {
              double obj_speed = get_sensor_fusion_speed(sensor_fusion[i]);
              double check_obj_s = get_sensor_fusion_future_position(sensor_fusion[i], prev_size);

              std::cout << "obj_speed: " << obj_speed << std::endl;
              std::cout << "check_obj_s: " << check_obj_s << std::endl;
              std::cout << "current_s: " << current_s << std::endl;
              if (
                // Is in front of me
                check_obj_s > current_s
              ) {
                
                // Is relatively close
                if ((check_obj_s - current_s) < 30) {
                  // Consider lane change when you can
                  lane_state = LaneState::WAIT;
                  // Match speed
                  speed_state = SpeedState::MATCH;
                }

                if (speed_state == SpeedState::MATCH) {
                  double adj = ((obj_speed * 2.24) - velocity_mph);
                  // Match the slowest object in our lane and ahead of us (and in sensor fusion range)
                  if (velocity_mph + adj < velocity_mph + speed_adjustment_mph) {
                    speed_adjustment_mph = adj;
                  }
                }

              }
            }
          }
          
          // Find gap to merge into
          if (lane_state == LaneState::WAIT) {
            // Pick best option: left, right, or stay
            // TODO(jafulfor): Do better cost functions here
            bool left_lane_better = lane != 0;
            bool right_lane_better = lane != 2; // Assuming 3 lanes!
            for(int i = 0; i < sensor_fusion.size(); i++) {
              // TODO(jafulfor): Eliminate left or right lanes
            }

            // Prefer left lane over right lane (in theory left lane is faster)
            if (left_lane_better) {
              lane -= 1;
              lane_state = LaneState::STAY;
              speed_state = SpeedState::FULL;
            } else if (right_lane_better) {
              lane += 1;
              lane_state = LaneState::STAY;
              speed_state = SpeedState::FULL;
            }
          }

          // Speed pacing (jerk)
          speed_adjustment_mph = min(speed_adjustment_mph, max_acceleration_mph);
          // Brake pacing (jerk)
          speed_adjustment_mph = min(-speed_adjustment_mph, -max_deceleration_mph);
          
          // Enforce max speed
          velocity_mph = min(velocity_mph + speed_adjustment_mph, 49.5);
          
          std::cout << "velocity_mph: " << velocity_mph << ", lane: " << lane << std::endl;
          std::cout << "lane_state: " << lane_state << ", speed_state: " << speed_state << std::endl;
          
          //
          // PLOT WAYPOINTS
          //

          // Define anchor points
          vector<double> pts_x;
          vector<double> pts_y;
          
          double ref_x;
          double ref_y;
          double ref_yaw;
          // Add initial waypoints for current projected position
          if (prev_size < 2) {
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
          
          // Compute future waypoints
          vector<double> next_waypoint;
          for (int i = 1; i < 4; i++) {
            next_waypoint = getXY(
              car_s + (i * 30),
              2 + (4 * lane),
              map_waypoints_s, map_waypoints_x, map_waypoints_y
            );
            pts_x.push_back(next_waypoint[0]);
            pts_y.push_back(next_waypoint[1]);
          }
          
          // Rotate pts to ref's perspective
          for (int i = 0; i < pts_x.size(); i++) {
            double shift_x = pts_x[i] - ref_x;
            double shift_y = pts_y[i] - ref_y;
            pts_x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
            pts_y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
          }
          
          //
          // INTERPOLATE ACTIONS
          //
          
          tk::spline path;
          
          path.set_points(pts_x, pts_y);
          
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for (int i = 0; i < prev_size; i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          double target_x = 30.0;
          double target_y = path(target_x);
          double target_d = distance(0, 0, target_x, target_y);
          // How many chunks needed to cover distance given target speed and chunk time is 0.02 seconds?
          double N = target_d / (0.02 * velocity_mph / 2.24);

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
//           for(int i = 0; i < next_x_vals.size(); i++) {
//             std::cout << i << ": (" << next_x_vals[i] << ", " << next_y_vals[i] << ")" << std::endl;
//           }
//           std::cout << "Sending response with points.size() " << next_x_vals.size() <<  std::endl;

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