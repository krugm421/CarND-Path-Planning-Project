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
#include "spline.h"

using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Transform a pose from car local coordinate system to global map perspective.
vector<double> car_to_world(double anchor_x, double anchor_y, double anchor_theta, double local_x, double local_y) {
    double global_x = anchor_x + (local_x * cos(anchor_theta)) - (local_y * sin(anchor_theta));
    double global_y = anchor_y + (local_x * sin(anchor_theta)) + (local_y * cos(anchor_theta));

    return {global_x, global_y};
}

// Transform a pose from global map perspective to car's local coordinate system (front of car faces positive x)
vector<double> world_to_car(double anchor_x, double anchor_y, double anchor_theta, double global_x, double global_y) {
	double local_x = (global_x - anchor_x) * cos(-anchor_theta) - (global_y - anchor_y) * sin(-anchor_theta);
	double local_y = (global_x - anchor_x) * sin(-anchor_theta) + (global_y - anchor_y) * cos(-anchor_theta);

    return {local_x, local_y};
}

// return closest waypoint
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWP = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);

		if(dist < closestLen)
		{
			closestLen = dist;
			closestWP = i;
		}

	}

	return closestWP;

}

// Return next waypoint, which is the closest waypoint that's in front (not in back)
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWP = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWP];
	double map_y = maps_y[closestWP];

	// heading from ego position to closest waypoint
	double heading = atan2((map_y-y),(map_x-x)); // radians

	double angle = fabs(theta-heading);  // difference between car's current heading and the heading to waypoint 
	angle = min(2*pi() - angle, angle);  // normalize

	// if more than 90 degrees angle to closest waypoint, i.e. it's behind you, then advance to next waypoint
	if(angle > pi()/4)
	{
		closestWP++;
		if (closestWP == maps_x.size())
			closestWP = 0;
	}

	return closestWP;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
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

#define MPH2MPS(i) (i * 0.44704F)
#define MPS2MPH(i) (i * 2.24F)
#define SF_VX   (3)
#define SF_VY   (4)
#define SF_S    (5)
#define SF_LANE (6)

#define FOLLOW_GAP (30.0F)
#define LANE_CHANGE_PREP_GAP (41.0F)
#define TIME_STEP (0.02F) // 0.02 seconds (50 Hz)
#define SPEED_LMT (49.5F) // speed limit

bool haveClearance(double sf_s, double sf_s_pred, double sf_v,
                    double car_s, double car_s_pred, double car_v, double clearance_gap)
{
    bool lane_clear = true;
    if ( (sf_s > (car_s - clearance_gap/3.0F)) && (sf_s < (car_s + clearance_gap)) ) {
        // The other car is within the protected space next us
        lane_clear = false;
//    } else if ((sf_s_pred > (car_s_pred - clearance_gap/3.0F)) && (sf_s_pred < (car_s_pred + clearance_gap))) {
//        // The other car's predicted location is within the predicted protected space next to us
//        lane_clear = false;
//    } else if ( (sf_s > car_s) && (sf_v < car_v) ) {
//        // The other car is in front of protected space but going slower
//        lane_clear = false;
    } else if ( (sf_s < (car_s - clearance_gap/3.0F)) && (sf_s > (car_s - clearance_gap/3.0F - 5))
            && (sf_v > (car_v + 10)) )  {
        // The other car is behind but close and approaching fast
        lane_clear = false;
        cout << "Approaching fast from BEHIND." << endl;
    }
    return lane_clear;
}