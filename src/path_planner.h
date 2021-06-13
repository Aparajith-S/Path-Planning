/// \file path_planner.h
/// \brief path planner component
/// \author s.aparajith@live.com
/// \date 30.05.2021
/// \copyright None reserved. MIT license
#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H
#include "sceneObjects.h"
#include "sceneObjects.h"
#include "trajectory.h"
#include "cost.h"
#include "interface.h"
#include "types.h"
#include <cstdint>
#include <vector>

/// \class Path Planner module
// points: uses some of the traffic rules from German traffic system. 
// Some rules are strictly followed such as
// 1. Rechtsfahrgebot - Drive on the right side only whenever it is possible
// 2. Rechtsueberholverbot - normally not allowed to overtake on the right
// 3. Rechtsueberholausnahme - not dealt in this project as it is applicable to > 3 marked lanes

namespace path_planner {

	/// \enum deals with planner states
	enum class State : uint8_t
	{
		INIT=0,
		RUN     // reach target speed! - 2 possibilities. either follow the 
				// vehicle in front or accelerate to reach set driver speed (= speed limit)
		        // Rechtsueberholverbot - no overtaking on the right 
	        	// Rechtsfahrgebot - move to right lane and prefer a right lane(right most of the current lanes)
	};

	/// \enum deals with the type of lane.
	enum class LaneType : uint8_t
	{
		OVERTAKING=0, // Overtaking lane to overtake slow moving traffic on the acceleration lane (Ueberholer)
		ACCELERATION, // Acceleration lane for driving at the max speed limit (Rechtsfahrgebot)
		ROUTINE  // the lane is for Trucks, routine driving (Rechtsfahrgebot) to drive at minimum permissible speed.
	};

	/// \fn operator overloading for comparing integer with lane type
	inline bool operator==(const int& lhs,const LaneType& rhs)
	{
		return (static_cast<LaneType>(lhs) == rhs);
	}

	/// \fn operator overloading for comparing integer with lane type
	inline bool operator==(const LaneType& lhs,const int& rhs)
	{
		return (static_cast<LaneType>(rhs) == lhs);
	}

	class PathPlanner
	{
	public:
		/// constructor
		explicit PathPlanner(interfaces::SnsFusionData& i_sensFusionData);
		virtual ~PathPlanner() {}
		
		/// \brief runnable main for the application 
		/// \param[in] i_input vehicle data 
		/// \param[in] mdata : map object
		/// \param[in] o_trajectory : generated trajectory
		/// \return chosen Target 
		Target process(interfaces::input const& i_input,
		path_planner::Map & mdata,
		interfaces::output& o_trajectory);

		/// \brief init for the application
		/// \param[in] i_input vehicle data 
		/// \param[in] mdata : map object 
		void init(interfaces::input const& i_input,path_planner::Map & mdata);
		
		/// \brief log the targets to the file
		/// \param[in] i_targets : list of targets 
		/// \param[in] i_input : input vehicle kinematics
		/// \param[in] cyclecount : ith cycle the application is running for
		void logTargetsToFile(std::vector<Target> const & i_targets,
		interfaces::input const & i_input,
		int cyclecount);
		
	private:
		State m_state;
		bool m_initialized;
		scene::objects m_objects;
		uint32_t m_cyclicCounter;
		std::vector<Target> m_targets;
		trajectory::trajectory m_trajectory;
		interfaces::TrajectoryFrenet m_prevPathsd;
	};
}

#endif