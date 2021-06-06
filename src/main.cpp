#include <uWS/uWS.h>
#include <fstream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"


// for convenience
using nlohmann::json;
using std::string;
using std::vector;

#define SPEEDLIMIT 22.0                   // Speedlimit in 22 m/s, roughly 50 mph 
#define SAMPLETIME 0.02                 // Sampletime in s
#define BASEDISTANCE SPEEDLIMIT*SAMPLETIME
#define MAXSPACING 100
#define LANEWIDTH 2.0                     // Lanewidth = 2 m
#define FRAMERATE 50
#define MAXS      6945.554
#define DESIREDPATHLENGTH 10
#define APPROACHDISTANCE 60.0                  // safety distance in m
#define FOLLOWDISTANCE 40.0
#define MAXACCELERATION 10.0                     // allowed maximal acceleration in m/s^2
#define LANECHANGEMARGIN 30.0

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

  int pathplannerState = 0; 
  // 0 = keep lane at 50 mph (default) 
  // 1 = follow target at save distance 
  // 2 = lane change left
  // 3 = lane change right
  int indexPreviousWaypoint = 0;

  // get spline object 
  tk::spline sObj;

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

  // desired lane 
  int lane = 1;
  // lanechange active 
  bool changingLanes = false;
  // reference velocity
  double v_ref = 0;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&pathplannerState,&indexPreviousWaypoint, &lane, &v_ref, &changingLanes]
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

            // printTelemetry(car_x, car_y, car_s, car_d, car_yaw, car_speed);

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

            vector<double> next_x_vals;
            vector<double> next_y_vals;

            /**
             * TODO: define a path made up of (x,y) points that the car will visit
             *   sequentially every .02 seconds
             * 
             * Interfaces: x/y point in world coordinate system, every point is 20ms appart
             */
            
            // Start Sensor Fusion
            int prev_size = previous_path_x.size();

            // get lane index of vehicle
            int lane_current = (int) floor(car_d/(2*LANEWIDTH));
            // approachingAgent anf followingAgent Flags
            bool approachingAgent = false;
            bool followingAgent = false;
            // lanechange left and right allowed flags
            bool lanechangeLeftAllowed = true;
            bool lanechangeRightAllowed = true;
            // distance to closest agent
            double sClosestAgent = MAXS;
            // prediction of s position of ego vehicle after
            double car_s_prediction_inc = ((double)prev_size * SAMPLETIME * car_speed * 0.44704);
            // iterate over sensor fusion/all non-ego vehicles/agent
            for (int i = 0; i < sensor_fusion.size(); i++)
            {
              float d = sensor_fusion[i][6];
              int lane_agent = (int) floor(d/(2*LANEWIDTH));
              // current state variables of the agent
              double vx_agent = sensor_fusion[i][3];
              double vy_agent = sensor_fusion[i][4];
              double v_agent = sqrt(vx_agent*vx_agent + vy_agent*vy_agent);
              double s_agent = sensor_fusion[i][5];
              // predict the agents s value for the remaining waypoints
              double s_agent_prediction_inc = ((double)prev_size * SAMPLETIME * v_agent);
              double s_agent_prediction = s_agent + s_agent_prediction_inc;
              
              if (lane_agent == lane)
              {
                // check for non-ego vehicles in the same lane

                // if agent is in front of me ...
                if (s_agent_prediction > car_s)
                {
                  if ((s_agent_prediction - car_s) < FOLLOWDISTANCE)
                  {
                    followingAgent = true;
                    approachingAgent = false;
                    sClosestAgent = s_agent;
                  }
                  else if (((s_agent_prediction - car_s) < APPROACHDISTANCE) && (car_speed * 0.44704 > v_agent))
                  {
                    approachingAgent = true;
                    sClosestAgent = s_agent;
                  }
                }
              }
              else if (abs(lane_agent - lane) == 1)
              {
                // calculate distance after one sample step
                double delta_s = 0;
                if (s_agent > car_s)
                {
                  delta_s = s_agent - car_s + s_agent_prediction_inc - car_s_prediction_inc;
                }
                else
                {
                  delta_s = car_s - s_agent - s_agent_prediction_inc + car_s_prediction_inc;
                }

                if(((lane_agent - lane) == -1) && delta_s < LANECHANGEMARGIN)
                {
                  // left
                  lanechangeLeftAllowed &= false;

                }
                else if (((lane_agent - lane) == 1) && delta_s < LANECHANGEMARGIN)
                {
                  // right
                  lanechangeRightAllowed &= false;
                }
                
              }
            }

            // Check if lanechange is finished. 
            if(changingLanes && (abs(LANEWIDTH*(2*lane + 1)) - car_d)/LANEWIDTH < 0.05){
              std::cout << "lane change done: " << lane << std::endl;
              changingLanes = false;
            }
            // End Sendor Fusion
            // Start Decision making
            // if approaching an agent try to change lanes and reduce speed till FOLLOWDISTANCE
            
            if (approachingAgent)
            {
              // if approaching an agent try to change lanes and reduce speed till FOLLOWDISTANCE
              if (lanechangeLeftAllowed && (lane == lane_current) && (lane != 0) && !changingLanes)
              {
                lane--;
                std::cout << "lane--: " << lane << std::endl;
                changingLanes = true;
              }
              else if (lanechangeRightAllowed && (lane == lane_current) && (lane != 2) && !changingLanes)
              {
                lane++;
                std::cout << "lane++: " << lane << std::endl;
                changingLanes = true;
              }
              else if ((sClosestAgent - car_s)> FOLLOWDISTANCE)
              {
                v_ref -= 0.1;
              }
            }
            else if (followingAgent)
            {
              if (lanechangeLeftAllowed && (lane == lane_current) && (lane != 0) && !changingLanes)  
              {
                lane--;
                std::cout << "lane--: " << lane << std::endl;
                changingLanes = true;
              }
              else if (lanechangeRightAllowed && (lane == lane_current) && (lane != 2)  && !changingLanes)
              {
                lane++;
                std::cout << "lane++: " << lane << std::endl;
                changingLanes = true;
              }
              else
              {
                v_ref -= 0.1;
              } 
            }
            else if(v_ref < SPEEDLIMIT)
            {
              v_ref += 0.1;
            }

            // std::cout << "Lanechange left/right allowed = " << lanechangeLeftAllowed << "  " << lanechangeRightAllowed << std::endl;
            // std::cout << "Current lane:  " << lane_current << std::endl;
            // End Decision making
            
            // Start Trajectory Planer
            // Those two variables will store the points the spline will be fitted to
            vector<double> pos_X;
            vector<double> pos_Y;
            vector<double> pos_S;
            // Create spline object
            tk::spline sObj;
            // vectors contianing the next path points, those will be sampled from the spline

            // location and orientation of the cars reference frame, used for tranformations later on
            double ref_x;
            double ref_y;
            double ref_yaw;

            // if left no or one left over points are remaining add the previous and current car position. Adding the previous car possition will ensure 
            // that transitions between splines will be smooth (1st + 2nd derivative are 0). Same applies to the els branch, in this case the last
            // and last - 1 point will be use to ensure smoothness  
            if (prev_size < 2)
            {
              // update reference frame
              ref_x = car_x;
              ref_y = car_y;
              ref_yaw = deg2rad(car_yaw);  
              // push last two points
              pos_X.push_back(car_x - cos(car_yaw));
              pos_X.push_back(car_x);
              pos_Y.push_back(car_y - sin(car_yaw));
              pos_Y.push_back(car_y);
            }
            else
            {
              // update reference frame
              ref_x = previous_path_x[prev_size - 1];
              ref_y = previous_path_y[prev_size - 1];

              double ref_x_prev = previous_path_x[prev_size - 2];
              double ref_y_prev = previous_path_y[prev_size - 2];

              ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);  
              // push last two points
              pos_X.push_back(ref_x_prev);
              pos_X.push_back(ref_x);
              pos_Y.push_back(ref_y_prev);
              pos_Y.push_back(ref_y);
            }

            // keep lane (default) 

            // add first all remaining path points
            for (int i = 0; i < prev_size; i++)
             {
               next_x_vals.push_back(previous_path_x[i]);
               next_y_vals.push_back(previous_path_y[i]);
             }
             
             // get the next 3 point along the map in the same lane...
             vector<double> spline_wp_0;
             vector<double> spline_wp_1;
             vector<double> spline_wp_2;
             if (lane != lane_current)
             {
               spline_wp_0 = getXY(car_s + 30, (1+2*lane)*LANEWIDTH, map_waypoints_s, map_waypoints_x, map_waypoints_y);
               spline_wp_1 = getXY(car_s + 60, (1+2*lane)*LANEWIDTH, map_waypoints_s, map_waypoints_x, map_waypoints_y);
               spline_wp_2 = getXY(car_s + 90, (1+2*lane)*LANEWIDTH, map_waypoints_s, map_waypoints_x, map_waypoints_y);
             }
             else
             {
               spline_wp_0 = getXY(car_s + 30, (1+2*lane)*LANEWIDTH, map_waypoints_s, map_waypoints_x, map_waypoints_y);
               spline_wp_1 = getXY(car_s + 60, (1+2*lane)*LANEWIDTH, map_waypoints_s, map_waypoints_x, map_waypoints_y);
               spline_wp_2 = getXY(car_s + 90, (1+2*lane)*LANEWIDTH, map_waypoints_s, map_waypoints_x, map_waypoints_y);
             }
              
             // ... add them to the spline fitting points
             pos_X.push_back(spline_wp_0[0]);
             pos_X.push_back(spline_wp_1[0]);
             pos_X.push_back(spline_wp_2[0]);

             pos_Y.push_back(spline_wp_0[1]);
             pos_Y.push_back(spline_wp_1[1]);
             pos_Y.push_back(spline_wp_2[1]);

             // transform spline fitting points to vehicle reference frame
             global2Local(pos_X, pos_Y, ref_x, ref_y, ref_yaw);

             // fit spline
             sObj.set_points(pos_X, pos_Y);

             // sample the spline for the next 30m
             double horizon_point_x = 30.0;
             double horizon_point_y = sObj(horizon_point_x);
             double horizon_dist = distance(0, horizon_point_x, 0, horizon_point_y);

             // adding remaining points to DESIREDPATHLENGTH
             double x_new = 0.0, y_new = 0.0;
             for (int i = 1; i <= DESIREDPATHLENGTH - prev_size; i++)
             {
               x_new = sampleTrajectory(sObj, SAMPLETIME*v_ref, x_new);
               y_new = sObj(x_new);

               double x_new_global = (x_new * cos(ref_yaw) - y_new * sin(ref_yaw));
               double y_new_global = (x_new * sin(ref_yaw) + y_new * cos(ref_yaw));

               x_new_global += ref_x; 
               y_new_global += ref_y;

               next_x_vals.push_back(x_new_global);
               next_y_vals.push_back(y_new_global);
             }
            // End Trajectory Planner
            // ... Pushing set points to the control ...
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