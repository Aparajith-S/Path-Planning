/// \file
/// \brief
/// \author s.aparajith@live.com
/// \copyright None reserved. MIT license
/// 
/// 
#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H
#include <cstdint>
#include<vector>
#include "interface.h"
#include "trajectory.hpp"
#include "sceneObjects.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Dense"
#include "Eigen-3.3/Eigen/LU"

/*
* points: uses some of the traffic rules from German traffic system. 
* Some rules are strictly followed such as
* 1. Rechtsfahrgebot - Drive on the right side only
* 2. Rechtsueberholverbot - normally not allowed to overtake on the right
* 3. Rechtsueberholausnahme - not dealt in this example as it is applicable to > 3 marked lanes
*/
namespace path_planner {

	enum class State : uint8_t
	{
		INIT,
		FOLLOW_LANE, // reach target speed! - 2 possibilities. either follow the 
				//vehicle in front or accelerate to reach set driver speed (= speed limit)
		OVERTAKE,//Rechtsueberholverbot - no right overtaking
		CHANGE_CENTER // Rechtsfahrgebot - move to right lane and prefer a right lane(right most of the center lanes)
	};
	struct egoState
	{
		double value;
		double value_dot;
		double value_ddot;
	};
	constexpr std::uint8_t kMaxHorizon = 50U; // distance Horizon for planning

	class PathPlanner
	{
	public:
		PathPlanner();
		virtual ~PathPlanner() {}
		void process(interfaces::input const& i_input, interfaces::output& o_trajectory);
		void PathPlanner::init(interfaces::input const& i_input);
		coefficients JMT(const std::vector<double>& i_start, const std::vector<double>& i_end, double T);
		std::pair< std::vector<double>, std::vector<double>> perturbedGoal(const std::vector<double>& i_goal_s, const std::vector<double>& i_goal_d);

	private:
		State m_state;
		bool m_initialized;
		double m_pos_x;
		double m_pos_y;
		double m_speed;
		double m_yaw;
		egoState m_s_start;
		egoState m_d_start;
		scene::objects m_objects;
	};
}

#endif