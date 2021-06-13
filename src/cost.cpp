/// \file cost.cpp
/// \brief cost function class definition for the trajectory generation
/// \author s.aparajith@live.com
/// \date 11.06.2021
/// \copyright None reserved. MIT license
#include "cost.h"
#include "sceneObjects.h"
#include "helpers.h"
#include "constants.h"
#include "types.h"
#include "geometry.h"
#include "interface.h"
#include <vector>
#include <cstdint>
#include <iostream>
#include <algorithm>
namespace path_planner{
using std::vector;
namespace internal{
/// @brief A function that returns a value between 0 and 1 
/// @details for x in the range[0, infinity] and -1 to 1 for x in the range[-infinity, infinity].
/// @param[in] i_x input.
/// @return logistic value between 0.0 and 1.0
inline double logistic(const double i_x)
{
    return (2.0 / (1 + exp(-i_x)) - 1.0);
}
}

Cost::Cost():
m_totalCost(0.0){}
Cost::~Cost() {}

/// @return saved cost that was recently computed
double Cost::get_cost() const{
  return m_totalCost;
}

/// @brief compute if two bounding boxes of a (x,y) and theta of two objects.
/// @see cost.h for more details
bool Cost::BoundingBoxOverlap(double x0, double y0,double theta0, double x1, double y1,double theta1) 
{
    rectangle bb1(params::kLengthBoundingBox,params::kWidthBoundingBox);
    rectangle bb2(params::kWidthBoundingBox,params::kWidthBoundingBox);
    bb1.rotateLongAxis(theta0);
    bb2.rotateLongAxis(theta1);
    bb1.translate(vector2d(x0,y0));
    bb2.translate(vector2d(x1,y1));
    return bb1.isSATOverlap(bb2);
}

/// @brief compute cost for collision
/// @see cost.h for more details
int Cost::checkCollisionOnTrajectory(interfaces::TrajectoryCart const &i_trajectory, std::map<uint8_t, std::vector<vector2d>> const & predictions)
{
    for(const auto & el : predictions)
  {
    uint8_t index = el.first;
    vector<vector2d> prediction = el.second;
    for (int i = 0; i < params::kSafeProximity; i++) 
    { 
      double obj_x = prediction[i].x;
      double obj_y = prediction[i].y;
      double obj_x_next = prediction[i+1].x;
      double obj_y_next = prediction[i+1].y;
      double obj_yaw = atan2(obj_y_next - obj_y, obj_x_next - obj_x);

      double ego_x = i_trajectory.x_vals[i];
      double ego_y = i_trajectory.y_vals[i];
      double ego_x_next = i_trajectory.x_vals[i+1];
      double ego_y_next = i_trajectory.y_vals[i+1];
      double ego_yaw = atan2(ego_y_next - ego_y, ego_x_next - ego_x);

      if (BoundingBoxOverlap(obj_x, obj_y, obj_yaw, ego_x, ego_y, ego_yaw)) 
      {
        m_log << (int)index << " collision at step " << i << std::endl;
        return (i+1);
      }
    }
  }
  return 0;
}

/// @brief compute cost based on max and total acceleration
/// @see cost.h for more details
bool Cost::maxAndTotalAcceleration(interfaces::TrajectoryCart const &i_trajectory)
{
  double total_acc=0.0;
  double max_acc=0.0;
  for (size_t t = 2; t < i_trajectory.x_vals.size(); t++) {
    double x   = i_trajectory.x_vals[t];
    double x_1 = i_trajectory.x_vals[t-1];
    double x_2 = i_trajectory.x_vals[t-2];
    double y   = i_trajectory.y_vals[t];
    double y_1 = i_trajectory.y_vals[t-1];
    double y_2 = i_trajectory.y_vals[t-2];
    double ax = (x - 2*x_1 + x_2) / (params::dT * params::dT);
    double ay = (y - 2*y_1 + y_2) / (params::dT * params::dT);
    double acc = sqrt(ax*ax + ay*ay);
    total_acc += acc * params::dT;
    if (acc > max_acc)
    {
      max_acc = acc;
    }
  }
  double total_acc_ps = total_acc_ps / (params::kMaxPoints * params::dT);
  if (ceil(max_acc) > params::kMaxAcc && total_acc_ps>params::kTotalAcc)
  {
    return true;
  } else {
    return false;
  }
}

/// @brief compute cost based on max and total jerk
/// @see cost.h for more details
bool Cost::maxAndTotalJerk(interfaces::TrajectoryCart const &i_trajectory)
{
  double total_jerk=0.0;
  for (size_t t = 3; t < i_trajectory.x_vals.size(); t++) 
  {
    double x   = i_trajectory.x_vals[t];
    double x_1 = i_trajectory.x_vals[t-1];
    double x_2 = i_trajectory.x_vals[t-2];
    double x_3 = i_trajectory.x_vals[t-3];

    double y   = i_trajectory.y_vals[t];
    double y_1 = i_trajectory.y_vals[t-1];
    double y_2 = i_trajectory.y_vals[t-2];
    double y_3 = i_trajectory.y_vals[t-3];
    // rounding to 2 decimals (cm precision) for numerical stability
    double jx = (x - 3*x_1 + 3*x_2 - x_3);
    jx = roundf(jx * 100.0) / 100.0;
    jx = jx / (pow(params::dT,3));
    double jy = (y - 3*y_1 + 3*y_2 - y_3);
    jy = roundf(jy * 100.0) / 100.0;
    jy = jy / (pow(params::dT,3));

    double jerk = sqrt(jx*jx + jy*jy);
    total_jerk += jerk * params::dT;
  }
  double jerk_per_second = total_jerk / (params::kMaxPoints * params::dT);
  if(jerk_per_second > params::kMaxJerk)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/// @brief computes the collective cost for a given \p i_trajectory
/// @see cost.h for more details
double Cost::computeCost(interfaces::TrajectoryCart const &i_trajectory, Target const & i_target, scene::objects const &i_objects, int car_lane)
{
  m_totalCost = 0.0;
  double CollisionCost = 0.0; // vs collisions, vs vehicle capabilities
  double cost_safety = 0; // vs buffer distance, vs visibility
  double cost_legality = 0; // vs speed limits
  double cost_comfort = 0; // vs jerk
  double cost_efficiency = 0; // vs desired lane and time to goal
  std::map<uint8_t, vector<vector2d>> predictions = i_objects.getPredictions();

  //cost function to prevent collision 
  CollisionCost = checkCollisionOnTrajectory(i_trajectory, predictions);
  
  cost_comfort = (double)maxAndTotalAcceleration(i_trajectory);
  cost_comfort += (double)maxAndTotalJerk(i_trajectory);
  //m_totalCost += params::kWeightEfficiency*maxVelocity(i_trajectory,i_objects,car_lane);
  m_totalCost += params::kWeightCollision*CollisionCost;
  m_totalCost+=params::kWeightLegal*cost_legality;
  m_totalCost+= params::kWeightComfort*cost_comfort;
  cost_efficiency = params::kPredictionHorizon - i_objects.getLaneFreeSpace(i_target.lane);
  m_totalCost = m_totalCost + params::kWeightEfficiency * cost_efficiency;
  return m_totalCost;
}
}