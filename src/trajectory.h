/// \file trajectory.h
/// \brief trajectory generation based on jerk minimizing trajectory equations
/// \author s.aparajith@live.com
/// \date 08.06.2021
/// \copyright None reserved. MIT license
#ifndef TRAJECTORY_H
#define TRAJECTORY_H
#include "cost.h"
#include "sceneObjects.h"
#include "interface.h"
#include "helpers.h"
#include "map.h"
#include "spline.h"
#include "logging.h"
#include<vector>
#include<tuple>
#include<array>
namespace path_planner {
namespace trajectory{

class trajectory
{
public:
explicit trajectory():m_cost(),m_initialized(false){}
virtual~trajectory() {}

/// \brief initializes the trajectory generator
/// \param[in] i_map Map object 
/// \param[in] iData ego vehicle data 
/// \return TrajectorySet with cartesian and frenet coordinate
interfaces::TrajectorySet init(path_planner::Map &i_map,interfaces::EgoData const & iData);

/// \brief main runnable of the trajectory generator
/// \param[in] i_input input data to the path planner
/// \param[in] mdata map data
/// \param[in] i_objects processed object data from the path planner
/// \param[in] io_targets list of all targets
/// \param[in] i_prev_path previous trajectory
/// \return tuple of Trajectory Cart and frenet with danger flag 
std::tuple<interfaces::TrajectoryCart,interfaces::TrajectoryFrenet,uint32_t,bool>
  process(interfaces::input const& i_input, 
  path_planner::Map & mdata,
  scene::objects const & i_objects,
  std::vector<Target> & io_targets,
  interfaces::PrevTrajectory const & i_prev_path);

/// \brief Jerk Minimalizing Trajectory coefficient generation
/// \param[in] i_start s and d frenet coordinates first second 
/// \param[in] i_end s and d frenet coordinates first second 
/// \param[in] T time till which the trajectory is to be generated
/// \return coefficients of the polynomial of order 5
interfaces::coefficients JMT(const std::vector<double>& i_start, const std::vector<double>& i_end, double T);

/// \brief uses Jerk Minimalizing Trajectory coefficients to generate trajectory based on waypoint
/// \param[in] i_target Target v and acceleration requested by the path planner
/// \param[in] ioMap map object 
/// \param[in] i_input input to the path planner
/// \param[in] i_prev previous trajectory with frenet and cartesian
/// \return TrajectorySet cartesian and frenet coordinates
interfaces::TrajectorySet generateTrajectoryJMT(Target const & i_target,
path_planner::Map & ioMap, 
interfaces::input const & i_input , 
interfaces::PrevTrajectory const & i_prev);

/// \brief gerenates frenet for a special case where time =0.0 to generate emergency braking maneuver
/// \param[in] i_target frenet for a special case where time =0.0 to generate emergency braking maneuver
/// \param[in] ioMap map object 
/// \param[in] i_input input to the path planner
/// \param[in] i_prev previous trajectory with frenet and cartesian
/// \return TrajectorySet cartesian and frenet coordinates
interfaces::TrajectorySet generateTrajectoryFrenet(Target const & i_target,
path_planner::Map &ioMap,
interfaces::input const & i_input,
interfaces::PrevTrajectory const & o_prev);
private:
bool m_initialized;
Cost m_cost;  // cost function object
logger m_log; // logging object
};
}
}


#endif