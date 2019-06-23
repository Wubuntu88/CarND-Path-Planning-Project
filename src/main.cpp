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
#include "utils.h"

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

  int lane = 1;
  double ref_vel = 0.0;


  h.onMessage([&ref_vel, &lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s, // @suppress("Invalid arguments")
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
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          bool too_close = false;
          bool emergency_close = false;
          bool can_turn_right = true;
          bool can_turn_left = true;
          double lead_car_velocity = 0.0;

          int prev_size = previous_path_x.size();

          int max_safe_follow_distance = 26; // car will not get closer than 18 meters
          int emergency_close_distance = 15;
          int lead_car_ahead_check_distance = 30;

          //for turning left and right.
          double safe_distance_ahead = 30.0;
          double safe_distance_behind = 22.0;

          lane = getLane(car_d);

          /*
           * Start of code to slow and speed up the vehicle
           */
          if(prev_size > 0) {
            car_s = end_path_s;
            car_d = end_path_d;
          }

          vector<double> lead_car = getCarInFront(lane, car_s, prev_size,
                                                  lead_car_ahead_check_distance, sensor_fusion); // @suppress("Invalid arguments")
          if(lead_car.size() > 0) {
            double car_in_front_s = lead_car[5];

            double check_car_s = getCheckCarS(lead_car, prev_size); // @suppress("Invalid arguments")

            // will only count cars in front
//            double distance_apart = car_in_front_s - car_s;
            double distance_apart = check_car_s - car_s;
            double lead_car_velocity_x = lead_car[3];
            double lead_car_velocity_y = lead_car[4];
            double lead_car_v = sqrt(lead_car_velocity_x*lead_car_velocity_x + lead_car_velocity_y*lead_car_velocity_y) * 2.24;

            if(distance_apart < max_safe_follow_distance) {
              too_close = true;
              lead_car_velocity = lead_car_v;
            }
            if(distance_apart < emergency_close_distance) {
              emergency_close = true;
            }

          }


          for(int i = 0; i < sensor_fusion.size(); ++i) {
            //if the car is in our lane
            vector<double> other_car = sensor_fusion[i];
            double other_car_s = other_car[5];
            double other_car_d = other_car[6];

            // calculates s value in the future
            double other_car_future_s = getCheckCarS(other_car, prev_size);

            int other_car_lane = getLane(other_car_d);

            bool other_car_is_too_close = other_car_future_s < car_s + safe_distance_ahead && other_car_future_s > car_s - safe_distance_behind;

            bool other_car_is_in_left_lane = lane - other_car_lane == 1;
            bool other_car_is_in_right_lane = other_car_lane - lane == 1;
            if (other_car_is_in_left_lane && other_car_is_too_close) {
              can_turn_left = false;
            }
            if (other_car_is_in_right_lane && other_car_is_too_close) {
              can_turn_right = false;
            }
          }

          if(too_close) {
            if (lane > 0 && can_turn_left) {
              lane -= 1;
              std::cout << "lane change left into lane: " << lane << std::endl;
              if(emergency_close) {
                std::cout << "emergency deceleration." << std::endl;
                ref_vel -= .33;
              }
            } else if (lane < 2 && can_turn_right){
              lane += 1;
              std::cout << "lane change right into lane: " << lane << std::endl;
              if(emergency_close) {
                std::cout << "emergency deceleration." << std::endl;
                ref_vel -= .33;
              }
            }

            else {
              if(emergency_close) {
              std::cout << "emergency deceleration." << std::endl;
              ref_vel -= .33;
              }
              else if(abs(lead_car_velocity - ref_vel) > .224) {
                std::cout << "decelerating, car in lane: " << lane << std::endl;
                ref_vel -= .224;
              }
              else {
                std::cout << "following, in lane: " << lane << std::endl;
                ref_vel = lead_car_velocity;
              }
            }
          }else {
            if((lane == 0 && can_turn_right) || (lane == 2 && can_turn_left)) {
              std::cout << "returning to mid lane" << std::endl;
              lane = 1;
            }
            else if (ref_vel < 49.5) {
              std::cout << "accelerating in lane: " << lane << std::endl;
              ref_vel += 0.224;
            }else {
              std::cout << "cruising in lane: " << lane << std::endl;
            }
          }


          /*
           * start of spline code
           */

          // Creates a list of widely spaced (x, y) waypoints, evenly spaced at 30m
          // Later we will interpolate these waypoints with a spline and fill it in with more points that control speed.
          vector<double> ptsx;
          vector<double> ptsy;

          /*
           * Start of getting reference variables
           */
          // reference variables
          int previous_path_size = previous_path_x.size();
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // if previous size is almost empty, use the car as a starting reference
          if(prev_size < 2) {
            // Use two points that make the path tangent to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          // use the previous path's end point as starting reference
          else {
            // Redefine reference state as previous path end point
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

            //Use two points that make the path tangent to the previous path's end point
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          //In Frenet add evenly 30m spaced points ahead of the starting reference
            vector<double> next_wp0 = getXY(car_s+30, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(car_s+60, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s+90, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);

            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);

            for(int i = 0; i < ptsx.size(); ++i) {
              //shift car reference angle to 0 degrees
              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;

              ptsx[i] = (shift_x * cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
              ptsy[i] = (shift_x * sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
            }

          // fit spline to center of lane
          tk::spline s;
          s.set_points(ptsx, ptsy);

          // Creates a list of widely spaced (x, y) waypoints, evenly spaced at 30m
          // Later we will interpolate these waypoints with a spline and fill it in with more points that control speed.
          // next path values
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // add previous path to next path vals
          for (int i = 0; i < prev_size; i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Calculate how to break up spline points so that we travel at our desired reference velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);
          double x_add_on = 0;

          for (int i = 0; i <= 50-previous_path_size; i++) {
            double N = (target_dist/(delta_t*ref_vel/2.24));
            double x_point = x_add_on + (target_x)/N;
            double y_point = s(x_point);
            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // rotate back to normal after rotating it earlier
            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          std::cout << "--------------" << std::endl;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

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

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, // @suppress("Invalid arguments")
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
