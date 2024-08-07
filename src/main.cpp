#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"  // Helper funcion to interpolate point
#include "utility.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Helper function from uWebSocket template
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

// Transform from Frenet s,d coordinates to Cartesian x,y
// Inverse from Frenet to Cartesian. It's not a linear transformation
// That something that's calculated at the very beginning that we can just feed it.
// And, that's used for the map inside the function itself
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};
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

  // Using stringstream to actually load that up
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

  // Car's lane. Starting at middle lane.
  int lane = 1;

  // Reference velocity.
  // Wnat to go as close to the speed limit as possible
  // But don't want to go over it
  double ref_vel = 0.0; // mph

  // Get passed the usual uWebSocket template
  // Actually get our localizaion data
  h.onMessage([&ref_vel, &lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy]
    (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {

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
		
		// All the variables that we're going to be working with in
		// order to decide how we want to be creating engineering points
        if (event == "telemetry") {
			// j[1] is the data JSON object
			//
			// UPDATE SENSOR FUSION
			//
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

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
			auto sensor_fusion = j[1]["sensor_fusion"];
			
			// Wich comes frome the previous path size
			// Help when I'm doing a transition
			// This is the last path that the car was following
			// before it does this particular run through of calculating more points
			int prev_size = previous_path_x.size();

			if(prev_size > 0)
			{
				car_s = end_path_s;
			}
			
			//
			// PREDICTION
			//
			// Analysing other cars positions.
			bool ahead_car = false;
            bool left_car = false;
            bool righ_car = false;
			
			//find ref_v to used
			for ( int i = 0; i < sensor_fusion.size(); i++ ) {
                
				// car is in my lane
				float d = sensor_fusion[i][6];
                int car_lane = -1;
				
                // is it on the same lane we are
                if ( d > 0 && d < 4 ) {
                  car_lane = 0;
                } else if ( d > 4 && d < 8 ) {
                  car_lane = 1;
                } else if ( d > 8 && d < 12 ) {
                  car_lane = 2;
                }
                if (car_lane < 0) {
                  continue;
                }
				
                // Find car speed.
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx*vx + vy*vy);
                double check_car_s = sensor_fusion[i][5];
				
                // Estimate car s position after executing previous trajectory.
                check_car_s += ((double)prev_size*0.02*check_speed);

                if ( car_lane == lane ) {
                  // Car in our lane.
                  ahead_car |= check_car_s > car_s && check_car_s - car_s < 30;
                } else if ( car_lane - lane == -1 ) {
                  // Car left
                  left_car |= car_s - 30 < check_car_s && car_s + 30 > check_car_s;
                } else if ( car_lane - lane == 1 ) {
                  // Car right
                  righ_car |= car_s - 30 < check_car_s && car_s + 30 > check_car_s;
                }
            }
			
			//
			// BEHAVIOR:
			//
			// To change the lane
            double speed_diff = 0;
            const double MAX_Limit_Speed = 49.5;
            const double MAX_Acceleration = .224;
            if ( ahead_car )
			{ // Car ahead
              if ( !left_car && lane > 0 )
			  {
                // if there is no car left and there is a left lane.
                lane--; // Change lane left.
              } 
			  else if ( !righ_car && lane != 2 )
			  {
                // if there is no car right and there is a right lane.
                lane++; // Change lane right.
              }
			  else 
			  {
                speed_diff -= MAX_Acceleration;
              }
            } 
			else 
			{
              if ( lane != 1 ) { // if we are not on the center lane.
                if ( ( lane == 0 && !righ_car ) || ( lane == 2 && !left_car ) ) 
				{
                  lane = 1; // Back to center.
                }
              }
              if ( ref_vel < MAX_Limit_Speed ) {
                speed_diff += MAX_Acceleration;
              }
            }
			
			//
			// TRAJETORY:
			//			
			// Create a list widely spaced (x,y) waypoints, evenly spaced at 30m
			// Later we will interpolate thes waypoints with a spline and fill it in with points that control spaced
			
			vector<double> ptsx;
			vector<double> ptsy;
			
			// Keep track of my reference state
			// Reference x,y yaw states
			// Either we will reference the starting point as where the car is or that previous paths end point
			double ref_x = car_x;
			double ref_y = car_y;
			double ref_yaw = deg2rad(car_yaw);
			
			// If previous size is almost empty, use the car as starting reference
			if(prev_size < 2)
			{
				// Use two point that make the path tangent to the car
				double prev_car_x = car_x - cos(car_yaw);
				double prev_car_y = car_y - sin(car_yaw);
				
				// Check out waht the previous path size was
				// It's either going to be pretty close to
				// empty or it's going to have some points that I can take advantage of.
				ptsx.push_back(prev_car_x);
				ptsx.push_back(car_x);
				
				ptsy.push_back(prev_car_y);
				ptsy.push_back(car_y);
			}
			// Use the previous path's end point as starting reference
			else
			{
				// Use the last two points.
				ref_x = previous_path_x[prev_size - 1];
                ref_y = previous_path_y[prev_size - 1];

				// Use two point that make the path tangent to the car
                double ref_x_prev = previous_path_x[prev_size - 2];
                double ref_y_prev = previous_path_y[prev_size - 2];
                ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

				// Use two points that make the path tangent to the previous path's end point
                ptsx.push_back(ref_x_prev);
                ptsx.push_back(ref_x);

                ptsy.push_back(ref_y_prev);
                ptsy.push_back(ref_y);
            }

            // Setting up target points in the future.
			// In Frenet add evenly 30m spaced points ahead of the starting reference
			// Instead of just looking at one distance increment, we looking out basically 35, 60, 90
			// And instead of looping through and creating 50
            vector<double> next_wp0 = getXY(car_s + 30, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(car_s + 60, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s + 90, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
 			
            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);
 
            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);
 			
            // Making coordinates to local car coordinates.
			// Shift car reference angle to 0 degrees
			// Trasnformation to this local car's coordinates
			// This is some of the math you might recognize from MPC
            for ( int i = 0; i < ptsx.size(); i++ ) 
			{
				// shift car reference angle to 0 degree
				double shift_x = ptsx[i] - ref_x;
				double shift_y = ptsy[i] - ref_y;

				ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
				ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
            }
			
			// Create the spline.
            tk::spline s;
			
			// Set (x,y) points to the spline
            s.set_points(ptsx, ptsy);

			// Output path points from previous path for continuity.
			// Define the actual (x,y) point we will use for the planner
          	vector<double> next_x_vals;
          	vector<double> next_y_vals;
			
			// Start with all of the previous path points from last time
            for ( int i = 0; i < prev_size; i++ )
			{
				next_x_vals.push_back(previous_path_x[i]);
				next_y_vals.push_back(previous_path_y[i]);
            }

            // Calculate distance y position on 30 m ahead.
			// Calculate to break up spline points so that we travel at our desired reference velocity
            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt(target_x*target_x + target_y*target_y);

            double x_add_on = 0;
	
			// Fill up the rest of our planner ofter filling it with previous points, 
			// here we always output 50 points
			for( int i = 1; i < 50 - prev_size; i++ )
			{
				// Adding on those points that are along the spline
				
				// To control Maximum Speed and Acceleration
				ref_vel += speed_diff;
				if ( ref_vel > MAX_Limit_Speed ) 
				{
					ref_vel = MAX_Limit_Speed;
				} 
				else if ( ref_vel < MAX_Acceleration )
				{
					ref_vel = MAX_Acceleration;
				}
				
				// Trying to figur out what N should be
				// Dividing it by 2.24 because I was saying this in miles per hour
				// it needs to be in meters per second
				double N = target_dist/(0.02*ref_vel/2.24);  // using reference velocity
				double x_point = x_add_on + target_x/N;
				double y_point = s(x_point);

				x_add_on = x_point;

				double x_ref = x_point;
				double y_ref = y_point;
				
				// Rotate back to normal after rotating it eariler
				x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
				y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

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
