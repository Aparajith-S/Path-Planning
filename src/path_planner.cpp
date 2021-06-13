/// \file path_planner.cpp
/// \brief path planner component
/// \author s.aparajith@live.com
/// \date 30.05.2021
/// \copyright None reserved. MIT license
#include "cost.h"
#include "path_planner.h"
#include "helpers.h"
#include "map.h"
#include "logging.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <algorithm>
#include <fstream>
using std::vector;
namespace path_planner {

PathPlanner::PathPlanner(interfaces::SnsFusionData& i_sensFusionData) :
  m_state(State::INIT),
  m_initialized(false),
  m_objects(i_sensFusionData),
  m_cyclicCounter(0U)
{}

/// \brief init for the application
/// \see path_planner.h for more details
void PathPlanner::init(interfaces::input const& i_input,path_planner::Map & mData)
{
  if (i_input.prevOp.previous_path_x.size() == 0) 
  {
    m_prevPathsd = m_trajectory.init(mData,i_input.currentData).frenet;
  }
    m_initialized=true;
}

/// \brief runnable main for the application 
/// \see path_planner.h for more details
Target PathPlanner::process(interfaces::input const& i_input,path_planner::Map & mdata, interfaces::output& o_trajectory)
	{
		//compute current parameters of the vehicle based on previous values 
		//get target speed for that lane.
		
		//get obstacles and their velocities within +/- the horizon.
		
		//set goal for all possible transitions
		//compute costs for possible transistions
		//generate minimum jerk trajectories using a quintic poly solver
		//update state.
    logger logout;
		//list of inputs
		auto const & car_s = i_input.currentData.car_s;
		auto const & car_d = i_input.currentData.car_d;
		auto const & car_x = i_input.currentData.car_x;
		auto const & car_y = i_input.currentData.car_y;
		auto const & car_yaw = i_input.currentData.car_yaw;
		auto const & car_speed = i_input.currentData.car_speed;
		auto const & car_lane = i_input.currentData.car_lane;
		auto const & car_danger = i_input.currentData.car_danger;
		interfaces::PrevTrajectory prev_path;
		prev_path.Prev_cart = {i_input.prevOp.previous_path_x,i_input.prevOp.previous_path_y};
		prev_path.Prev_frenet = m_prevPathsd;
		prev_path.prev_reused_xy =std::min((int)i_input.prevOp.previous_path_x.size(),params::kMaxPoints);
		m_objects = scene::objects(i_input.snsFusion);
		m_objects.refreshData(i_input.snsFusion,i_input.currentData);
  	m_objects.predict();
		m_objects.computeSafeProximities(i_input.currentData);
		m_objects.setLaneParams(i_input.currentData);
		auto ego_lane = helper::getLane(car_d);// route this to everyone!
 		Target target{};
  	target.time = 2.0;
  	double car_speed_target =  i_input.currentData.car_target_speed;
    double car_target_acc =0.1;
		double safety_distance =  m_objects.getSafeProximity();
		m_targets.clear();
    State nextstate = m_state;
    switch(m_state)
    {
      case State::INIT: 
        car_speed_target=m_objects.getSafeLaneSpeed(ego_lane);
        car_target_acc = params::kMaxAcc;
        target.velocity=car_speed_target;
        target.acc=car_target_acc;
        target.lane=ego_lane;
        target.time=2.0;
        m_targets.push_back(target);
        nextstate = State::RUN;
        break;
      case State::RUN:   
      if((m_objects.getSafeDistanceFront(ego_lane)>=25.0) &&
          m_objects.getSafeLaneSpeed(ego_lane)>car_speed)
        { //basically like a cruise control 
          car_speed_target=params::kMaxHwySpeedLimit;
          if((car_speed_target)> car_speed)                               
            car_target_acc = 0.7*params::kMaxAcc;
          else if(car_speed_target < car_speed)
            car_target_acc = -0.7*params::kMaxAcc;
          logout<<"cruise";
        }
      else if(m_objects.getSafeLaneSpeed(ego_lane)<car_speed)
          {
            //spacing control.
            //stay in any lane.
            double front_speed=m_objects.getSafeLaneSpeed(ego_lane);
            if(front_speed>(car_speed+2.0))
            {
              car_speed_target = front_speed;
              car_target_acc= 0.3*params::kMaxAcc;
              logout<<"spacing1";
            }
            else if(front_speed<car_speed)
            {
              car_speed_target = front_speed;
              car_target_acc= -0.3*params::kMaxAcc;
              logout<<"spacing2";
            }
            else
            {
              car_speed_target = car_speed;
              car_target_acc= 0.0;
              logout<<"spacing3";
            }
          }
          else
          {
            car_speed_target=car_speed;
            car_target_acc=0.0;
            logout<<"cruise with current speed";
          }
          target.acc=car_target_acc;
          target.velocity=car_speed_target;
          target.time=2.0;
          target.lane=ego_lane;
          m_targets.push_back(target);
          //handle different lane cases.
          if(ego_lane == LaneType::OVERTAKING)
          {
            double front_speed=m_objects.getSafeLaneSpeed(ego_lane);
            double right_speed=m_objects.getSafeLaneSpeed(ego_lane+1U);
            double free_space_on_right = m_objects.getLaneFreeSpace(ego_lane+1U);
            double free_space_on_current = m_objects.getLaneFreeSpace(ego_lane);
            target.velocity= car_speed;//we want to change lane without change in velocity
            target.acc = 0.0;
            target.time = 2.0;
            target.lane =  static_cast<uint8_t>(LaneType::ACCELERATION);
            if(car_speed<right_speed && free_space_on_right>free_space_on_current)
            {
              m_targets.push_back(target);
              logout<<"acceleraion chg lane";
            }
          }
          else if(ego_lane == LaneType::ACCELERATION)
          {
            double front_speed=m_objects.getSafeLaneSpeed(ego_lane);
            double left_speed=m_objects.getSafeLaneSpeed(ego_lane-1U);
            double right_speed=m_objects.getSafeLaneSpeed(ego_lane+1U);
            double free_space_on_right = m_objects.getLaneFreeSpace(ego_lane+1U);
            double free_space_on_current = m_objects.getLaneFreeSpace(ego_lane);
            double free_space_on_left = m_objects.getLaneFreeSpace(ego_lane-1U);
            target.acc = 0.0;
            target.time = 2.0;
            //move to routine lane
            if(car_speed<right_speed && free_space_on_right>free_space_on_current)
            {
              target.velocity= car_speed;//we want to change lane without change in velocity
              target.lane =  static_cast<uint8_t>(LaneType::ROUTINE);
              m_targets.push_back(target);
              logout<<"routine chg lane";
            }
            if(car_speed<left_speed && free_space_on_left>free_space_on_current)
            {
              target.velocity= left_speed;//we want to overtake with a preferably faster speed.
              target.lane =  static_cast<uint8_t>(LaneType::OVERTAKING);
              m_targets.push_back(target);
              logout<<"overtaking chg lane";
            }
          }
          else if(ego_lane == LaneType::ROUTINE)
          {
              double front_speed=m_objects.getSafeLaneSpeed(ego_lane);
              double left_speed=m_objects.getSafeLaneSpeed(ego_lane-1U);
              double free_space_on_current = m_objects.getLaneFreeSpace(ego_lane);
              double free_space_on_left = m_objects.getLaneFreeSpace(ego_lane-1U);
              target.velocity= car_speed;//we want to change lane without change in velocity
              target.acc = 0.0;
              target.time = 2.0;
              if(car_speed<left_speed && free_space_on_left>free_space_on_current)
            {
              target.lane =  static_cast<uint8_t>(LaneType::ACCELERATION);
              m_targets.push_back(target);
              logout<<"overtaking chg lane";
            }
          }
      break;
      default: break;
    }
  m_state=nextstate;
  //Emergency deceleration in an unfortunate situation where 
  // collision cannot be avoided
  target.acc=-params::kEmergencyDec;
  target.time=0.0;
  target.velocity=0.0;
  target.lane=ego_lane;
  m_targets.push_back(target);
  if(m_targets.size())
  {
    auto chosen = m_trajectory.process(i_input,mdata,m_objects,m_targets,prev_path);
    m_prevPathsd = std::get<1>(chosen);
    o_trajectory.next_x_vals= std::get<0>(chosen).x_vals;
    o_trajectory.next_y_vals= std::get<0>(chosen).y_vals;
    //logTrajsToFile(m_targets,i_input,m_cyclicCounter);
    m_objects.log_objs_to_file(i_input,m_cyclicCounter);
    ++m_cyclicCounter;
    logout<< "!!!!! target: velocity=" << m_targets[std::get<2>(chosen)].velocity << " accel=" << m_targets[std::get<2>(chosen)].acc << '\n';
    return m_targets[std::get<2>(chosen)];
  }
  return Target();
}

/// \brief log the targets to the file
/// \see path_planner.h for more details
void PathPlanner::logTargetsToFile(std::vector<Target> const & i_targets, interfaces::input const & i_input,int cyclecount)
{
  std::ofstream fout;
  if(cyclecount==0)
  {
      fout.open("trajOpLog.csv", std::ofstream::out);
      fout<<"cycle,tgt.nr.,t.vel,t.acc,t.cost,t.time"<<std::endl;
  }
  else
  {
      fout.open("trajOpLog.csv", std::ofstream::out|std::ofstream::app);
  }
  for(int i=0;i<i_targets.size();i++)
  {
    fout<<cyclecount<<','<<i<<","<<i_targets[i].velocity<<","
    <<i_targets[i].acc<<","<<i_targets[i].cost<<","<<i_targets[i].time<<std::endl;
  }   	
  fout.close();
}

}