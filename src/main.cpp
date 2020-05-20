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
  const double max_s = 6945.554;

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

  // Start in lane 1 (middle)
  // Index notes: left ~ 0, middle ~ 1, right ~ 2
  int lane = 1;

  // Have a reference velocity to target (< speed limit)
  // Initiallize with 0.0
  double ref_vel = 0.0; //MPH
  const double kMaxSpeed = 49.5;
  const double kMaxAcc = 0.224;  // ~ 5m/s^2

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
   &map_waypoints_dx,&map_waypoints_dy,&lane,&ref_vel,&kMaxSpeed,&kMaxAcc]
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
          /*
          Get information of our car from the simulator
          */
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
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Get number of points that were not executed by the previous trajectory
          int prev_traj_size = previous_path_x.size();

          // If there are any unprocessed points, set car_s to the end s value
          if(prev_traj_size > 0) {
            car_s = end_path_s;
          }

          /*
          Get information of other cars on the same side of the road from the simulator
          */
          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          /*
          PREDICTION
          */
          // Set flags that indicate other cars on left/right/current lanes too close
          bool too_close_ahead = false;
          bool too_close_left = false;
          bool too_close_right = false;

          // Loop over all detected cars (in sensor_susion)
          // sensor_fusion data format: [id, x, y, vx, vy, s, d]
          for (int i=0; i < sensor_fusion.size(); i++) {
            double detected_d = sensor_fusion[i][6];
            int detected_lane = -1;
            if ((0 < detected_d) && (detected_d < 4)) {
              detected_lane = 0;
            } else if ((4 < detected_d) && (detected_d < 8)) {
              detected_lane = 1;
            } else if ((8 < detected_d) && (detected_d < 12)) {
              detected_lane = 2;
            } else {
              // Ignore the detected car if it's not on the same side of the road with our car
              continue;
            }

            double detected_vx = sensor_fusion[i][3];
            double detected_vy = sensor_fusion[i][4];
            double detected_v = sqrt(detected_vx * detected_vx + detected_vy * detected_vy);

            double detected_s = sensor_fusion[i][5];
            // Predict the position of the detected car
            detected_s += (double)prev_traj_size * detected_v * 0.02;

            // Set true if there is a car within 30m ahead in current lane
            // or in a range of +- 30m in left/right lane

            if (detected_lane == lane) {
              too_close_ahead |= (detected_s > car_s) && (detected_s < car_s + 30.0);
            } else if (detected_lane == (lane - 1)) {
              too_close_left |= (detected_s > (car_s - 30.0)) && (detected_s < (car_s + 30.0));
            } else if (detected_lane == (lane + 1)) {
              too_close_right |= (detected_s > (car_s - 30.0)) && (detected_s < (car_s + 30.0));
            }
          }

          /*
          BEHAVIOR PLANNING
          */

          double delta_vel = 0.;
          if (too_close_ahead) {
            // Check to change lane
            if ((!too_close_left) && (lane > 0)) {
              // Our car can safely change to the left lane
              lane--;
            } else if ((!too_close_right) && (lane < 2)) {
              // Our car can safely change to the right lane
              lane++;
            } else {
              // Our car can't change lane because of dangerous
              // Let our car slow down
              delta_vel -= kMaxAcc;
            }
          } else {
            // There is no car ahead in the dangerous range (+30 meters)
            // Check the center lane and change to the central lane if possible
            if (lane != 1) {
              if (((lane == 0) && (!too_close_right)) || ((lane == 2) && (!too_close_left))) {
                lane = 1;
              }
            }
            if (ref_vel < kMaxSpeed) {
              delta_vel += kMaxAcc;
            }
          }

          /*
          TRAJECTORY GENERATION (using spline libs: https://kluge.in-chemnitz.de/opensource/spline/)
          */
          // The trajectories points
          std::vector<double> pts_x;
          std::vector<double> pts_y;

          // Define variables to define the last car state
          // Current state is given by car_x, car_y and car_yaw (meters and degrees)
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          if (prev_traj_size < 2) {
            // A dummy point
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            // Add the dummy point and the current point to the trajectories points
            pts_x.push_back(prev_car_x);
            pts_y.push_back(prev_car_y); 

            pts_x.push_back(car_x);
            pts_y.push_back(car_y);
          } else {
            // Take the last two non-processed points
            ref_x = previous_path_x[prev_traj_size-1];
            ref_y = previous_path_y[prev_traj_size-1];

            double ref_x_prev = previous_path_x[prev_traj_size-2];
            double ref_y_prev = previous_path_y[prev_traj_size-2];

            // Calculate yaw
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

            // Add the last two non-processed points to the trajectories points
            pts_x.push_back(ref_x_prev);
            pts_y.push_back(ref_y_prev);
            pts_x.push_back(ref_x);
            pts_y.push_back(ref_y);
          }

          double next_lane_d = 4*lane + 2;
          // Add 3 target points of the trajectory: (+30m, +60m, +90m in the s axis in Frenet coordinate)
          // convert the three points from Frenet to cartesian coordinates
          std::vector<double> next_wp1 = getXY(car_s + 30, next_lane_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          std::vector<double> next_wp2 = getXY(car_s + 60, next_lane_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          std::vector<double> next_wp3 = getXY(car_s + 90, next_lane_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          // Push converted points
          pts_x.push_back(next_wp1[0]);
          pts_x.push_back(next_wp2[0]);
          pts_x.push_back(next_wp3[0]);

          pts_y.push_back(next_wp1[1]);
          pts_y.push_back(next_wp2[1]);
          pts_y.push_back(next_wp3[1]);

          // Transform the waypoints from global coordinates to our car coordinates
          // Shifting the car orientation to 0 degrees
          for (int i = 0; i < pts_x.size(); i++) {
            // Computed shifted coordinates
            double shifted_x = pts_x[i] - ref_x;
            double shifted_y = pts_y[i] - ref_y;
            
            pts_x[i] = shifted_x*cos(0-ref_yaw) - shifted_y*sin(0-ref_yaw);
            pts_y[i] = shifted_x*sin(0-ref_yaw) + shifted_y*cos(0-ref_yaw);
          }

          // Create a spline
          tk::spline s;

          // Compute spline to fit the trajectories points
          s.set_points(pts_x, pts_y);

          vector<double> next_x_vals;
          vector<double> next_y_vals;
          // Add non-processed points to next trajectory
          for (int i = 0; i<prev_traj_size; i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          double target_x = 30.0;
          double target_y = s(target_x); 
          double target_dist = sqrt(target_x * target_x + target_y * target_y);



          double x_prev = 0; // x offset to generate points
          for (int i=1; i < 50-prev_traj_size; i++) {
            ref_vel += delta_vel;
            if (ref_vel > kMaxSpeed) {
              ref_vel = kMaxSpeed;
            }
            else if (ref_vel < kMaxAcc) {
              ref_vel = kMaxAcc;
            }
            double N = target_dist / (0.02 * ref_vel / 2.24);
            // Get point using spline
            double x_point = x_prev + target_x / N;
            double y_point = s(x_point);

            // Update x_prev (offset)
            x_prev = x_point;

            // temporary variables in order not to use modified x_point, y_point values
            double x_point_temp = x_point;
            double y_point_temp = y_point;

            x_point = x_point_temp*cos(ref_yaw) - y_point_temp*sin(ref_yaw);
            y_point = x_point_temp*sin(ref_yaw) + y_point_temp*cos(ref_yaw);

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          json msgJson;
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