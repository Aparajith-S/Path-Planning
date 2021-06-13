/// \file map.h
/// \brief map handler with cubic spline interpolation
/// \author s.aparajith@live.com
/// \date 05.06.2021
/// \copyright None reserved. MIT license
#ifndef MAP_H
#define MAP_H
#include <math.h>
#include <iostream>
#include <fstream>
#include <vector>
#include "spline.h"
namespace path_planner{
class Map {

public:
  Map() {};
  virtual ~Map(){}

  /// @brief gets the frenet coordinates based on the map data.
  /// @param[in] x of cartesian coordinates
  /// @param[in] y of cartesian coordinates
  /// @param[in] theta yaw angle in cartesian coordinates
  /// @return vector of size 2 containing s and d
  std::vector<double> getFrenet(double x, double y, double theta);
  
  /// @brief gets the frenet coordinates based on the map data.
  /// @details from the coarse (not course) data of map
  /// @param[in] s of cartesian coordinates
  /// @param[in] d of cartesian coordinates
  /// @return vector of size 2 containing x and y
  std::vector<double> getXY(double s, double d);

  /// @brief gets the frenet coordinates based on the map data.
  /// @details from the fine resolution data of map
  /// @param[in] s of cartesian coordinates
  /// @param[in] d of cartesian coordinates
  /// @return vector of size 2 containing x and y
  std::vector<double> getXYspline(double s, double d);

  /// @brief gets the frenet speed based on the map data.
  /// @details from the fine resolution data of map
  /// @param[in] Vxy of cartesian coordinates
  /// @param[in] s of cartesian coordinates
  /// @return Speed in frenet.
  double getSpeedToFrenet(double Vxy, double s);

  /// @brief reads the map data.
  /// @param[in] map_file : filename string. 
  void read(std::string map_file);

private:
  /// @brief compute closest waypoint in the map
  /// @param [in] x :cartesian coordinate
  /// @param [in] y :cartesian coordinate
  /// @param [in] maps_x :cartesian waypoints vector 
  /// @param [in] maps_y :cartesian waypoints vector 
  int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
  
  /// @brief compute next waypoint in the map
  /// @param [in] x :cartesian coordinate
  /// @param [in] y :cartesian coordinate
  /// @param [in] theta :yaw angle in cartesian coordinate system
  /// @param [in] maps_x :cartesian waypoints vector 
  /// @param [in] maps_y :cartesian waypoints vector  
  int NextWaypoint(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
  tk::spline spline_x;
  tk::spline spline_y;
  tk::spline spline_dx;
  tk::spline spline_dy;
  std::vector<double> map_waypoints_x;
  std::vector<double> map_waypoints_y;
  std::vector<double> map_waypoints_s;
  std::vector<double> map_waypoints_dx;
  std::vector<double> map_waypoints_dy;
  std::vector<double> map_s; // pre-computed for faster access

  // better granularity: 1 point per meter
  std::vector<double> new_map_waypoints_x;
  std::vector<double> new_map_waypoints_y;
  std::vector<double> new_map_waypoints_dx;
  std::vector<double> new_map_waypoints_dy;
  std::vector<double> new_map_s; // pre-computed for faster access
};
}
#endif // MAP_H
