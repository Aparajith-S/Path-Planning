/// \file cost.h
/// \brief cost function class definition for the trajectory generation
/// \author s.aparajith@live.com
/// \date 11.06.2021
/// \copyright None reserved. MIT license
#ifndef COST_H
#define COST_H
#include "types.h"
#include "interface.h"
#include "helpers.h"
#include "sceneObjects.h"
#include "logging.h"
#include <cmath>
#include <vector>
#include <map>
#include <cmath>
namespace path_planner{
/// \class cost function generation
class Cost {
public:
  Cost();
  virtual ~Cost();
  
  /// @brief computes the collective cost for a given \p i_trajectory
  /// @param[in] i_trajectory cartesian coordinates trajectory
  /// @param[in] i_target target detail for the given trajectory
  /// @param[in] i_objects processed object data from the path planner
  /// @param[in] car_lane lane information of the ego vehicle
  /// @return cost for the trajectory \p i_trajectory
  double computeCost(interfaces::TrajectoryCart const &i_trajectory, 
  Target const & i_target, 
  scene::objects const &i_objects, 
  int car_lane);

  /// @return saved cost that was recently computed
  double get_cost() const;
private:
  
  /// @brief compute if two bounding boxes of a (x,y) and theta of two objects.
  /// @param[in] x0 : object 1  x coordinate in cartesian system
  /// @param[in] y0 : object 1  y coordinate in cartesian system
  /// @param[in] theta0 : object 1  yaw angle in cartesian system
  /// @param[in] x1 : object 2  x coordinate in cartesian system
  /// @param[in] y1 : object 2  y coordinate in cartesian system
  /// @param[in] theta1 : object 2  yaw angle in cartesian system
  /// @return true if the boxes overlap otherwise false.
  bool BoundingBoxOverlap(double x0, double y0,double theta0, double x1, double y1,double theta1);
  
  /// @brief compute cost based on max and total acceleration
  /// @param[in] traj : cartesian coordinates of the trajectory for which the cost is computed
  /// @return true when either max or total acceleration is exceeding the limits
  bool maxAndTotalAcceleration(interfaces::TrajectoryCart const &traj);
  
  /// @brief compute cost based on max and total jerk
  /// @param[in] traj : cartesian coordinates of the trajectory for which the cost is computed
  /// @return true when either max or total jerk is exceeding the limits
  bool maxAndTotalJerk(interfaces::TrajectoryCart const &traj);
  
  /// @brief compute cost for collision
  /// @param[in] traj : cartesian coordinates of the trajectory for which the cost is computed
  /// @param[in] predictions : map of objects with predicted trajectories into the future. 
  /// @return a value increasing with distance where the collision would occur. 
  int  checkCollisionOnTrajectory(struct interfaces::TrajectoryCart const &trajectory, 
  std::map<uint8_t,std::vector<vector2d>> const &predictions);
  double m_totalCost;
  logger m_log;
};

}
#endif // COST_H
