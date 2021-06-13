/// \file map.cpp
/// \brief path planner component
/// \author s.aparajith@live.com
/// \date 05.06.2021
/// \copyright None reserved. MIT license
#include "helpers.h"
#include "constants.h"
#include <vector>
#include <iostream>
#include <sstream>
#include <math.h>
#include "map.h"
#include <time.h>
using namespace std;

namespace path_planner{
double MAX_S;

/// @brief reads the map data.
/// @see map.h for more details
void Map::read(string map_file) 
{
  ifstream in_map_(map_file.c_str(), ifstream::in);
  string line;
  bool not_started = true;
  double x0, y0, dx0, dy0;
  double last_s = 0;

  // Load up map values for waypoint's x,y,s and d vectors
  while (getline(in_map_, line)) {
    istringstream iss(line);
  	double x, y;
  	float s, d_x, d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
    if (not_started) {
      x0 = x; y0 = y; dx0 = d_x; dy0 = d_y;
      not_started = false;
    }

    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    last_s = s;
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }
  assert(map_waypoints_x.size() && "map not loaded, probably path include is missing");

  MAX_S = params::kMapEnd;
    map_waypoints_x.push_back(x0);
    map_waypoints_y.push_back(y0);
    map_waypoints_s.push_back(MAX_S);
    map_waypoints_dx.push_back(dx0);
    map_waypoints_dy.push_back(dy0);

  spline_x.set_points(map_waypoints_s, map_waypoints_x);
  spline_y.set_points(map_waypoints_s, map_waypoints_y);
  spline_dx.set_points(map_waypoints_s, map_waypoints_dx);
  spline_dy.set_points(map_waypoints_s, map_waypoints_dy);

  // remove last point so we do not have duplicates (x,y):
  // it was just for spline continuity at wraparound
    map_waypoints_x.pop_back();
    map_waypoints_y.pop_back();
    map_waypoints_s.pop_back();
    map_waypoints_dx.pop_back();
    map_waypoints_dy.pop_back();


  double len_ref = 0;
  double prev_x = spline_x(0);
  double prev_y = spline_y(0);
  for (double s = 1; s <= floor(MAX_S); s++) {
    double x = spline_x(s);
    double y = spline_y(s);
    len_ref += helper::distance(x, y, prev_x, prev_y);
    prev_x = x;
    prev_y = y;
  }
  // map with higher precision: 
  // 1 point every 1 meter (instead of every 30 meters)
  new_map_waypoints_x;
  new_map_waypoints_y;
  new_map_waypoints_dx;
  new_map_waypoints_dy;
  for (double s = 0; s <= floor(MAX_S); s++) {
    double x = spline_x(s);
    double y = spline_y(s);
    double dx = spline_dx(s);
    double dy = spline_dy(s);

    new_map_waypoints_x.push_back(x);
    new_map_waypoints_y.push_back(y);
    new_map_waypoints_dx.push_back(dx);
    new_map_waypoints_dy.push_back(dy);
  }

	double frenet_s = 0.0;
  map_s.push_back(0.0);
	for (size_t i = 1; i < map_waypoints_x.size(); i++) {
		frenet_s += helper::distance(map_waypoints_x[i], map_waypoints_y[i], map_waypoints_x[i-1], map_waypoints_y[i-1]);
    map_s.push_back(frenet_s);
	}

	frenet_s = 0.0;
  new_map_s.push_back(0.0);
  // new map: make 1 point every meter : increased resolution
	for (size_t i = 1; i < new_map_waypoints_x.size(); i++) {
		frenet_s += helper::distance(new_map_waypoints_x[i], new_map_waypoints_y[i], new_map_waypoints_x[i-1], new_map_waypoints_y[i-1]);
    new_map_s.push_back(i);
	}
}

/// @brief compute closest waypoint in the map
/// @see map.h for more details
int Map::ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y) {

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

  int size = maps_x.size();

  if (size <= 200) {
	  for (int i = 0; i < size; i++) {
	  	double map_x = maps_x[i];
	  	double map_y = maps_y[i];
	  	double dist = helper::distance(x,y,map_x,map_y);
	  	if(dist < closestLen) {
	  		closestLen = dist;
	  		closestWaypoint = i;
	  	}
	  }
  } else  { // Faster search with big maps: 2 hierarchical steps of search
    // 1) Search a point relatively close to the nearest
    int jump_points = size / 181; // so that we have 1 jump_points with a 181 points map (default)
    int point = 0;
	  while(point < size)
    {
	  	double map_x = maps_x[point];
	  	double map_y = maps_y[point];
	  	double dist = helper::distance(x,y,map_x,map_y);
	  	if(dist < closestLen) {
	  		closestLen = dist;
	  		closestWaypoint = point;
	  	}
      point += jump_points;
    }

    // 2) Search a point which is the nearest in a refined area
	  //for (int i = closestWaypoint - 181; i < closestWaypoint + 181; i++)
	  for (int i = closestWaypoint - 91; i < closestWaypoint + 91; i++) {
      int idx = i;
      if (i < 0) {
        idx += size;
      } else if (i >= size) {
        idx -= size;
      }

	  	double map_x = maps_x[idx];
	  	double map_y = maps_y[idx];
	  	double dist = helper::distance(x,y,map_x,map_y);
	  	if(dist < closestLen) {
	  		closestLen = dist;
	  		closestWaypoint = idx;
	  	}
    }
  }
	return closestWaypoint;
}

/// @brief compute next waypoint in the map
/// @see map.h for more details
int Map::NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y) {
	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = fabs(theta-heading);
	angle = min(2*M_PI - angle, angle); // XXX bug fix

	if(angle > M_PI/4)
	{
		closestWaypoint++;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0; // XXX bug fix
    }
	}
	return closestWaypoint;
}

/// @brief gets the frenet coordinates based on the map data.
/// @see map.h for more details
vector<double> Map::getFrenet(double x, double y, double theta) {
  vector<double> &maps_s = this->new_map_s; ; 
  vector<double> &maps_x = this->new_map_waypoints_x;
  vector<double> &maps_y = this->new_map_waypoints_y;

	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0) {
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

	double frenet_d = helper::distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point
	  double center_x = 1000 - maps_x[prev_wp];
	  double center_y = 2000 - maps_y[prev_wp];
	  double centerToPos = helper::distance(center_x,center_y,x_x,x_y);
	  double centerToRef = helper::distance(center_x,center_y,proj_x,proj_y);
	  if (centerToPos <= centerToRef) {
	  	frenet_d *= -1;
	  }

  double frenet_s = maps_s[prev_wp]; // XXX faster
	frenet_s += helper::distance(0,0,proj_x,proj_y);

  assert(frenet_d >= 0);

	return {frenet_s, frenet_d};
}

/// @brief gets the frenet coordinates based on the map data.
/// @details from the coarse (not course) data of map
/// @see map.h for more details
vector<double> Map::getXY(double s, double d) {
  vector<double> &maps_s = map_waypoints_s; ; 
  vector<double> &maps_x = map_waypoints_x;
  vector<double> &maps_y = map_waypoints_y;

	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) )) {
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-M_PI/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};
}

/// @brief gets the frenet coordinates based on the map data.
/// @details from the fine resolution data of map
/// @see map.h for more details
vector<double> Map::getXYspline(double s, double d) {
  //for JMT wraparound
  s = fmod(s, MAX_S); 
	double x = spline_x(s) + d * spline_dx(s);
	double y = spline_y(s) + d * spline_dy(s);

	return {x,y};
}

/// @brief gets the frenet speed based on the map data.
/// @details from the fine resolution data of map
/// @see map.h for more details
double Map::getSpeedToFrenet(double Vxy, double s) {
  s = fmod(s, MAX_S);
  double dx_over_ds = spline_x.deriv(1, s);
  double dy_over_ds = spline_y.deriv(1, s);
  double Vs = (Vxy / sqrt(dx_over_ds*dx_over_ds + dy_over_ds*dy_over_ds));
  return Vs;
}

}