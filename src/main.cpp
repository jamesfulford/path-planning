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

double get_sensor_fusion_s(vector<double> sensor_fusion) {
  return sensor_fusion[5];
}

double get_sensor_fusion_future_position(vector<double> sensor_fusion, int prev_size) {
  return get_sensor_fusion_s(sensor_fusion) + (double) prev_size * 0.02 * get_sensor_fusion_speed(sensor_fusion);
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

bool is_ahead_of (double them, double me, double total) {
  // TODO(jafulfor): Fix to account for wrap-around
  return (me - them) < 0;
}

enum Cost {
  // In terms of MPH
  ILLEGAL = 2048, // off-road
  BLOCKED = 1024, // close to my position (I'm potentially already in the lane)
  BLOCKED_AHEAD = 8, // will be behind another car
  RIGHT = 0, // penalty for how many lanes to the right (prefer left lanes)
  RELUCTANCE = 3, // penalty if lane is not current lane

  OPTION_BONUS = 6 // bonus per lane shift option lane provides
};

bool is_lane_legal (int lane) {
  return (lane < 3 && lane >= 0);
}

double lane_shift_option_bonus (int lane) {
  return (
    // add 1 if left lane is legal
    is_lane_legal(lane - 1) ? Cost::OPTION_BONUS : 0
    // add 1 if right lane is legal
    + is_lane_legal(lane + 1) ? Cost::OPTION_BONUS : 0
  );
}

double rightness_penalty (int lane) {
  return lane * Cost::RIGHT;
}

vector<vector<double>> objects_in_lane(vector<vector<double>> sensor_fusion, int lane) {
  vector<vector<double>> relevant;
  for (int i = 0; i < sensor_fusion.size(); i++) {
    if (
      get_sensor_fusion_lane(sensor_fusion[i]) == lane
    ) {
      relevant.push_back(sensor_fusion[i]);
    }
  }
  return relevant;
}

vector<vector<double>> filter_min_s (vector<vector<double>> sensor_fusion, double min_s, int steps_into_future, double max_s) {
  vector<vector<double>> relevant;
  for (int i = 0; i < sensor_fusion.size(); i++) {
    double sf_s = get_sensor_fusion_future_position(sensor_fusion[i], steps_into_future);
    if (
      is_ahead_of(sf_s, min_s, max_s)
    ) {
      relevant.push_back(sensor_fusion[i]);
    }
  }
  return relevant;
}

vector<vector<double>> filter_max_s (vector<vector<double>> sensor_fusion, double top_s, int steps_into_future, double max_s) {
  vector<vector<double>> relevant;
  for (int i = 0; i < sensor_fusion.size(); i++) {
    double sf_s = get_sensor_fusion_future_position(sensor_fusion[i], steps_into_future);
    if (
      is_ahead_of(top_s, sf_s, max_s)
    ) {
      relevant.push_back(sensor_fusion[i]);
    }
  }
  return relevant;
}

double min_speed_in_lane(vector<vector<double>> sensor_fusion) {
  double min_speed = 49.5;
  for (int i = 0; i < sensor_fusion.size(); i++) {
    min_speed = min(min_speed, get_sensor_fusion_speed(sensor_fusion[i]));
  }
  return min_speed;
}

double cost_of_lane(int lane, vector<vector<double>> sensor_fusion, double current_s, int steps_into_future, double max_s) {
  double cost = 0.0;

  // ILLEGAL exit
  if (!is_lane_legal(lane)) {
    // Short circuit
    return Cost::ILLEGAL;
  }

  bool blocked_ahead = false;
  vector<vector<double>> objects_in_this_lane = objects_in_lane(
    filter_min_s(
      filter_max_s(
        sensor_fusion,
        current_s + 100.0,
        steps_into_future,
        max_s
      ),
      current_s - 15.0,
      steps_into_future,
      max_s
    ),
    lane
  );
  for (int i = 0; i < objects_in_this_lane.size(); i++) {
    double sf_s = get_sensor_fusion_future_position(objects_in_this_lane[i], steps_into_future);
    // BLOCKED exit
    if (
      // Me + 25 is ahead of them (they are closer than 25 ahead)
      is_ahead_of(current_s + 15.0, sf_s, max_s)
      // They are ahead of me - 15 (they are closer than 15 behind)
      && is_ahead_of(sf_s, current_s - 10.0, max_s)
    ) {
      // Short circuit
      return Cost::BLOCKED;
    }

    blocked_ahead = blocked_ahead || (
      is_ahead_of(sf_s, current_s, max_s)
      && !is_ahead_of(sf_s, current_s + 50.0, max_s)
    );
  }

  cost += blocked_ahead ? Cost::BLOCKED_AHEAD : 0; // penalize lanes that have cars close ahead
  cost += rightness_penalty(lane); // prefer left lanes

  cost -= min_speed_in_lane(objects_in_this_lane); // reward lanes where I can go faster
  cost -= lane_shift_option_bonus(lane); // reward lanes where I get more switching options

  return cost;
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

  h.onMessage([&max_s,&velocity_mph,&lane,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
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

          // Match speed
          double target_speed = min_speed_in_lane(
            objects_in_lane(
              filter_min_s(
                filter_max_s(
                  sensor_fusion,
                  current_s + 30.0,
                  prev_size,
                  max_s
                ),
                current_s, // don't match speed of objects behind you!
                prev_size,
                max_s
              ),
              lane
            )
          );
          double speed_adjustment_mph = target_speed - velocity_mph;

          //
          // Decide lane
          //
          // Shift left
          double left_cost = cost_of_lane(lane - 1, sensor_fusion, current_s, prev_size, max_s) + Cost::RELUCTANCE;
          // Stay
          double middle_cost = cost_of_lane(lane, sensor_fusion, current_s, prev_size, max_s);
          // Shift right
          double right_cost = cost_of_lane(lane + 1, sensor_fusion, current_s, prev_size, max_s) + Cost::RELUCTANCE;

          if (
            left_cost < middle_cost
            && left_cost < right_cost
          ) {
            lane -= 1;
          } else if (
            right_cost < left_cost
            && right_cost < middle_cost
          ) {
            lane += 1;
          }

          // Speed pacing (jerk)
          speed_adjustment_mph = min(speed_adjustment_mph, max_acceleration_mph);
          // Brake pacing (jerk)
          speed_adjustment_mph = -min(-speed_adjustment_mph, -max_deceleration_mph);

          // Enforce max speed
          velocity_mph = min(velocity_mph + (speed_adjustment_mph / 2.24), 49.5);

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
              car_s + (i * 45),
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

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

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
