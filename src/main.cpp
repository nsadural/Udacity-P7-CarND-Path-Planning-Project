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

  // Define constant limits for vehicle
  const double V_MAX = 49.5;  // max allowable speed in mph
  const double A_MAX = 5.0; // max allowable acceleration magnitude in m/s^2

  // Starting lane
  int lane = 1;

  // Initial state
  string state = "KL";

  // Target velocity to reference throughout simulation [mph]
  double v_ref = 0.0;

  // Initial v_ref for PLCL/PLCR
  double v_ref_L = V_MAX;
  double v_ref_R = V_MAX;

  h.onMessage([&state,&v_ref,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&lane,&V_MAX,&A_MAX,&v_ref_L,&v_ref_R]
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

          json msgJson;

          // **********DEBUG********** //
          //std::cout << "\ncar_s: " << car_s << "\tcar_d: " << car_d << "\tcar_speed: " << car_speed;

          // Size of previous path
          int prev_size = previous_path_x.size();

          // Collision avoidance using sensor fusion data
          if (prev_size > 0) {

              // Use previous path's endpoint s value to avoid future collision
              car_s = end_path_s;

          }

          bool too_close = false;
          double check_speed = 0.0;

          // For each other car sensed in proximity...
          for (int i = 0; i < sensor_fusion.size(); i++) {

              // If other car is in my lane...
              float d = sensor_fusion[i][6];
              if (d < (2 + 4 * lane + 2) && d >(2 + 4 * lane - 2)) {
                  double vx = sensor_fusion[i][3];
                  double vy = sensor_fusion[i][4];
                  double check_car_s = sensor_fusion[i][5];

                  check_speed = sqrt(vx * vx + vy * vy);

                  // Predict other car's future s value by projecting current s value
                  check_car_s += ((double)prev_size * 0.02 * check_speed);

                  // If other car's projected s is ahead of my car and gap is <30 m...
                  if ((check_car_s > car_s) && ((check_car_s - car_s) < 30)) {
                      too_close = true;

                      break;
                  }
              }
          }

          double cost_PLCL = 1.0;
          double cost_PLCR = 1.0;

          /****************************** KL to PLCL *******************************************/
          if ((state.compare("KL") == 0) && too_close && (lane > 0)) {

              int no_car_ahead = 1;
              int lane_left = lane - 1;
              double check_speed_left = 0.0;
              bool lane_clear = true;
              bool set_speed = true;

              // For each other car sensed in proximity...
              for (int i = 0; i < sensor_fusion.size(); i++) {

                  // If other car is in lane to left of current lane...
                  float d = sensor_fusion[i][6];
                  if (d < (2 + 4 * lane_left + 2) && d >(2 + 4 * lane_left - 2)) {
                      double vx = sensor_fusion[i][3];
                      double vy = sensor_fusion[i][4];
                      double check_car_s = sensor_fusion[i][5];

                      check_speed_left = sqrt(vx * vx + vy * vy);

                      // Predict other car's future s value by projecting current s value
                      check_car_s += ((double)prev_size * 0.02 * check_speed_left);

                      // Convert from m/s to mph
                      check_speed_left = 2.24 * check_speed_left;

                      // If other car's projected s is ahead of my car and gap is <100 m...
                      if ((check_car_s > car_s) && ((check_car_s - car_s) < 100)) {
                          no_car_ahead = 0;
                          
                          if (set_speed) {
                              v_ref_L = check_speed_left;
                              set_speed = false;
                          }

                          // If left lane speed is slower than current lane speed
                          if (check_speed_left < check_speed * 2.24) {
                              lane_clear = false;
                          }
                      }

                      // If other car's projected s is in the lane gap I want to enter...
                      if (((check_car_s - car_s) < 30) && ((check_car_s - car_s) > -10)) {
                          lane_clear = false;
                      }

                      
                  }
              }

              // Calculate PLCL cost function
              if (lane_clear) {
                  cost_PLCL = 1 / (100 * no_car_ahead + check_speed_left);
              }
          }

          /********************************** KL to PLCR *****************************************/
          if ((state.compare("KL") == 0) && too_close && (lane < 2)) {

              int no_car_ahead = 1;
              int lane_right = lane + 1;
              double check_speed_right = 0.0;
              bool lane_clear = true;
              bool set_speed = true;

              // For each other car sensed in proximity...
              for (int i = 0; i < sensor_fusion.size(); i++) {

                  // If other car is in lane to left of current lane...
                  float d = sensor_fusion[i][6];
                  if (d < (2 + 4 * lane_right + 2) && d >(2 + 4 * lane_right - 2)) {
                      double vx = sensor_fusion[i][3];
                      double vy = sensor_fusion[i][4];
                      double check_car_s = sensor_fusion[i][5];

                      check_speed_right = sqrt(vx * vx + vy * vy);

                      // Predict other car's future s value by projecting current s value
                      check_car_s += ((double)prev_size * 0.02 * check_speed_right);

                      // Convert from m/s to mph
                      check_speed_right = 2.24 * check_speed_right;

                      // If other car's projected s is ahead of my car and gap is <100 m...
                      if ((check_car_s > car_s) && ((check_car_s - car_s) < 100)) {
                          no_car_ahead = 0;

                          if (set_speed) {
                              v_ref_R = check_speed_right;
                              set_speed = false;
                          }

                          // If right lane speed is slower than current lane speed
                          if (check_speed_right < check_speed * 2.24) {
                              lane_clear = false;
                          }
                      }

                      // If other car's projected s is in the lane gap I want to enter...
                      if (((check_car_s - car_s) < 30) && ((check_car_s - car_s) > -10)) {
                          lane_clear = false;
                      }
                  }
              }

              // Calculate PLCR cost function
              if (lane_clear) {
                  cost_PLCR = 1 / (100 * no_car_ahead + check_speed_right);
              }
          }

          /******************************* KL ********************************************/
          if (state.compare("KL") == 0) {
              // Update v_ref based on collision avoidance for KL state
              // If I am within 30m...
              if (too_close) {
                  if (v_ref > 2.24 * check_speed * 0.95) {
                      v_ref -= 0.02 * 2.24 * A_MAX * 0.8;
                  }
                  else if (v_ref < 2.24 * check_speed * 0.80) {
                      v_ref += 0.02 * 2.24 * A_MAX * 0.8;
                  }
              }
              // If I'm not too close...
              else if (v_ref < V_MAX) {
                  v_ref += 0.02 * 2.24 * A_MAX; // Speed up
              }
          }

          // Compare PLCL/PLCR cost functions to determine PLCL/PLCR state and v_ref
          if ((cost_PLCL < 1.0) || (cost_PLCR < 1.0)) {
              if (cost_PLCL < cost_PLCR) {
                  state = "PLCL";
              }
              else {
                  state = "PLCR";
              }
          }

          /*********************************** PLCL *******************************************/
          if (state.compare("PLCL") == 0) {
              if (v_ref < v_ref_L) {
                  v_ref += 0.02 * 2.24 * A_MAX;
              }
              else {
                  state = "LCL";
                  v_ref_L = V_MAX;
                  v_ref_R = V_MAX;
              }
          }

          /*********************************** PLCR ********************************************/
          if (state.compare("PLCR") == 0) {
              if (v_ref < v_ref_R) {
                  v_ref += 0.02 * 2.24 * A_MAX;
              }
              else {
                  state = "LCR";
                  v_ref_L = V_MAX;
                  v_ref_R = V_MAX;
              }
          }

          /*********************************** LCL **********************************************/
          if (state.compare("LCL") == 0) {
              lane = lane - 1;
              state = "KL";
          }
          
          /*********************************** LCR **********************************************/
          if (state.compare("LCR") == 0) {
              lane = lane + 1;
              state = "KL";
          }

          // **********DEBUG********** //
          //std::cout << "\tv_ref: " << v_ref;
          //std::cout << "\tstate: " << state;
          //std::cout << "\tcost_PLCL: " << cost_PLCL << "\tcost_PLCR: " << cost_PLCR;

          // Create vectors for evenly, widely spaced (x,y) waypoints that will be splined later
          vector<double> px;
          vector<double> py;

          // Set reference x, y, yaw (EITHER where car is OR the previous path's endpoint)
          double ref_x = car_x;
          double ref_y = car_y;
          double angle = deg2rad(car_yaw);
          double ref_yaw = angle;

          // Generate (x,y) waypoints for spline...
          // if the previous path is almost empty...
          if (prev_size < 2) {
              
              double prev_car_x = car_x - cos(ref_yaw);
              double prev_car_y = car_y - sin(ref_yaw);

              px.push_back(prev_car_x);
              px.push_back(car_x);

              py.push_back(prev_car_y);
              py.push_back(car_y);

          } 
          // or use the previous path's endpoint
          else {

              // Previous path's endpoint
              ref_x = previous_path_x[prev_size - 1];
              ref_y = previous_path_y[prev_size - 1];

              // Point before previous path's endpoint
              double prev_ref_x = previous_path_x[prev_size - 2];
              double prev_ref_y = previous_path_y[prev_size - 2];

              //Calculate reference yaw angle from the previous path's last two points
              ref_yaw = atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);

              px.push_back(prev_ref_x);
              px.push_back(ref_x);

              py.push_back(prev_ref_y);
              py.push_back(ref_y);

          }

          // Add three waypoints using coordinates mapped from the Frenet to Cartesian frame
          vector<double> next_waypoint_0 = getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_waypoint_1 = getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_waypoint_2 = getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          px.push_back(next_waypoint_0[0]);
          px.push_back(next_waypoint_1[0]);
          px.push_back(next_waypoint_2[0]);

          py.push_back(next_waypoint_0[1]);
          py.push_back(next_waypoint_1[1]);
          py.push_back(next_waypoint_2[1]);

          // Shift car's reference angle to zero degrees and shift (x,y) accordingly
          for (int i = 0; i < px.size(); i++) {

              double shift_x = px[i] - ref_x;
              double shift_y = py[i] - ref_y;

              px[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
              py[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);

          }

          // Create a spline and set points
          tk::spline sp;
          sp.set_points(px, py);

          // Create vectors of (x,y) points to be used for path planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // First, add all leftover previous path points that were not used
          for (int i = 0; i < previous_path_x.size(); i++) {

              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);

          }

          // Calculate parameters for accessing spline points for traveling at desired speed
          double target_x = 30.0;
          double target_y = sp(target_x);
          double target_dist = sqrt(target_x * target_x + target_y * target_y);
          double x_add_on = 0;

          for (int i = 1; i <= 50 - previous_path_x.size(); i++) {

              double N = target_dist / (0.02 * v_ref / 2.24); // Number of points to target distance to maintain constant v_ref (0.02 sec/point, 2.24 mph/(m/s))
              double x_p = x_add_on + (target_x / N);
              double y_p = sp(x_p);

              x_add_on = x_p; // Update for the next iteration

              double x_ref = x_p;
              double y_ref = y_p;
              
              // Shift (x,y) back from zero degrees to corresponding yaw angle
              x_p = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
              y_p = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

              x_p += ref_x;
              y_p += ref_y;

              next_x_vals.push_back(x_p);
              next_y_vals.push_back(y_p);

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