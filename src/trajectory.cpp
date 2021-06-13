/// \file trajectory.h
/// \brief trajectory generation based on jerk minimizing trajectory equations
/// \author s.aparajith@live.com
/// \date 08.06.2021
/// \copyright None reserved. MIT license
#include"trajectory.h"
#include "polyfunction.h"
#include "cost.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Dense"
#include "Eigen-3.3/Eigen/LU"
#include "spline.h"
#include <iostream>
#include <vector>
#include <algorithm>
namespace path_planner{
namespace trajectory{
using std::vector;
using helper::getXY;
using helper::mph2ms;
using helper::deg2rad;
using helper::getFrenet;
using helper::getFrenetDfromLane;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;
using interfaces::dfrenet;
using interfaces::coefficients;
/// \brief initializes the trajectory generator
/// \see trajectory.h for more details
interfaces::TrajectorySet trajectory::init(path_planner::Map & mdata,interfaces::EgoData const & iData)
{
    //init splines once. 
    if(!m_initialized)
    {   interfaces::TrajectorySet traj;
        std::vector<dfrenet> spath_s(params::kMaxPoints, dfrenet{0.0, 0.0, 0.0});
        std::vector<dfrenet> spath_d(params::kMaxPoints, dfrenet{0.0, 0.0, 0.0});
        for (int i = 0; i < params::kMaxPoints; i++) 
        {
            spath_s[i] = dfrenet{iData.car_s, 0, 0};
            spath_d[i] = dfrenet{iData.car_d, 0, 0};
        }
        traj.frenet.s_vals = spath_s;
        traj.frenet.d_vals = spath_d;
        m_initialized=true;
        return traj;
    }
}

/// \brief main runnable of the trajectory generator
/// \see trajectory.h for more details
std::tuple<interfaces::TrajectoryCart,interfaces::TrajectoryFrenet,uint32_t,bool> trajectory::process(
interfaces::input const& i_input,
path_planner::Map & mdata, 
scene::objects const & i_objects,
std::vector<Target> & io_targets,
interfaces::PrevTrajectory const & i_prev_path)
{   
    auto const & car_lane = i_input.currentData.car_lane;
    std::vector<interfaces::TrajectoryCart> returnValueCart;
    std::vector<interfaces::TrajectoryFrenet> returnValueFrenet;
    std::vector<double> cost_vector;
for (std::uint32_t i = 0; i < io_targets.size(); i++) 
{
    interfaces::TrajectorySet sTrajectory;
    if (params::kUseJMT==true) 
    {
      // generate JMT trajectory in s and d: converted then to (x,y) for trajectory output
    if (io_targets[i].time <= 0.0)  
        {sTrajectory = generateTrajectoryFrenet(io_targets[i],mdata, i_input, i_prev_path);
         m_log<<"Traj_frenet"<<std::endl;
        }
    else 
        {sTrajectory = generateTrajectoryJMT(io_targets[i], mdata,i_input, i_prev_path);
         m_log<<"Traj_JMT"<<std::endl;
        }

      returnValueFrenet.push_back(sTrajectory.frenet);
    }
    
    // do cost computations
    double cost = m_cost.computeCost(sTrajectory.cart,io_targets[i],i_objects,car_lane);
    cost_vector.push_back(cost);
    io_targets[i].cost=cost;//update cost
    returnValueCart.push_back(sTrajectory.cart);
}
  
  // find the lowest cost trajectory
  double min_cost_ = std::numeric_limits<double>::max();
  uint32_t min_cost_index = 0;
  for (size_t i = 0; i < cost_vector.size(); i++) {
    if (cost_vector[i] < min_cost_) {
      min_cost_ = cost_vector[i];
      min_cost_index = i;
    }
  }

  // danger condition. confusing traffic situation led to a unsolvable mess.
  if (min_cost_ >= params::kWeightCollision) 
  {
    min_cost_index = cost_vector.size() - 1;
    min_cost_ = cost_vector[min_cost_index];
  }
  bool emergency;
  if (io_targets[min_cost_index].time <= 0.0) 
  {
    io_targets[min_cost_index].danger = true;
  } 
  else 
  {
    io_targets[min_cost_index].danger = false;
  }

  if (io_targets[min_cost_index].danger) {
    m_log<< "Emergency Braking activated!";
  }
  return std::make_tuple(returnValueCart[min_cost_index],returnValueFrenet[min_cost_index],min_cost_index,emergency); 
}

/// \brief uses Jerk Minimalizing Trajectory coefficients to generate trajectory based on waypoint
/// \see trajectory.h for more details
interfaces::TrajectorySet trajectory::generateTrajectoryJMT(Target const & i_target,path_planner::Map &ioMap, interfaces::input const & i_input , interfaces::PrevTrajectory const  & i_prev)
{
    interfaces::TrajectorySet returnValue;
    //generate minimum jerk trajectories using a quintic poly solver
    //update state.
    //list of inputs
    auto const & car_s = i_input.currentData.car_s;
    auto const & car_d = i_input.currentData.car_d;
    auto const & car_x = i_input.currentData.car_x;
    auto const & car_y = i_input.currentData.car_y;
    auto const & car_yaw = i_input.currentData.car_yaw;
    auto const & car_speed = i_input.currentData.car_speed;

    //prev values
    auto const &prev_path_x = i_prev.Prev_cart.x_vals;
    auto const &prev_path_y = i_prev.Prev_cart.y_vals;
    auto const &prev_path_s = i_prev.Prev_frenet.s_vals;
    auto const &prev_path_d = i_prev.Prev_frenet.d_vals;  
    //outputs
    auto & next_x_vals = returnValue.cart.x_vals;
    auto & next_y_vals = returnValue.cart.y_vals;
    auto & next_s_vals = returnValue.frenet.s_vals;
    auto & next_d_vals = returnValue.frenet.d_vals;

    int prev_size = i_prev.prev_reused_xy;
    vector<dfrenet> newpath_s(params::kMaxPoints,dfrenet{0.0,0.0,0.0});
    vector<dfrenet> newpath_d(params::kMaxPoints,dfrenet{0.0,0.0,0.0});
    int lastpoint;
    if(params::kMaxPointsReused<params::kMaxPoints)
    {
        lastpoint= params::kMaxPoints-prev_path_x.size() + prev_size -1;
    }
    else
    {
        lastpoint = params::kMaxPoints-1;
    }

    double si, si_dot=0, si_ddot;
    double di, di_dot, di_ddot;
    si      = prev_path_s[lastpoint].value;
    si_dot  = prev_path_s[lastpoint].value_dot;
    si_ddot = prev_path_s[lastpoint].value_ddot;

    di      = prev_path_d[lastpoint].value;
    di_dot  = prev_path_d[lastpoint].value_dot;
    di_ddot = prev_path_d[lastpoint].value_ddot;

    double sf, sf_dot, sf_ddot;
    double df, df_dot, df_ddot;
    if(i_target.velocity<=10)
    {
        df_ddot =  0;
        df_dot  =  0;
        df      = di;

        sf_ddot = 0;
        sf_dot  = mph2ms(i_target.velocity);

        // XXX
        sf_dot = std::min(sf_dot, si_dot + 10.0 * params::kMaxSpeedIncrement);
        sf_dot = std::max(sf_dot, si_dot - 10.0 * params::kMaxSpeedIncrement);

        sf = si + 2 * sf_dot * i_target.time;
    }
    else
    {
        df_ddot = 0;
        df_dot  = 0;
        df      = helper::get_dcenter(i_target.lane);

        sf_ddot = 0;
        sf_dot = mph2ms(i_target.velocity);
        // we use JMT for lane changes only
        // no need to try to reach amx speed during lane changes
        sf_dot = std::min(sf_dot, 0.9 * params::kMaxHwySpeedLimitms);

        // XXX just in case ...
        sf_dot = std::min(sf_dot, si_dot +  10 * params::kMaxSpeedIncrement);
        sf_dot = std::max(sf_dot, si_dot - 10 * params::kMaxSpeedIncrement);
        sf = si + sf_dot * i_target.time;
    }
        vector<double> start_s = { si, si_dot, si_ddot};
        vector<double> end_s = { sf, sf_dot, 0};

        vector<double> start_d = { di, di_dot, di_ddot };
        vector<double> end_d = { df, df_dot, df_ddot};
        vector<double> poly_s = JMT(start_s, end_s, i_target.time);
        vector<double> poly_d = JMT(start_d, end_d, i_target.time);
        for (int i = 0; i < prev_size; i++) 
        {
            newpath_s[i] = prev_path_s[params::kMaxPoints - prev_path_x.size() + i];
            newpath_d[i] = prev_path_d[params::kMaxPoints - prev_path_x.size() + i];
            next_x_vals.push_back(prev_path_x[i]);
            next_y_vals.push_back(prev_path_y[i]);
        }
        double t = params::dT;
        for (int i = prev_size; i < params::kMaxPoints; i++) 
        { function eqn_s;
            function eqn_d;
            eqn_s.constructEquation(poly_s);
            eqn_d.constructEquation(poly_d);
            double s = eqn_s(t);
            double s_dot = eqn_s.differentiate()(t);
            double s_ddot = eqn_s.differentiate().differentiate()(t);
            double d = eqn_d(t);
            double d_dot = eqn_d.differentiate()(t);
            double d_ddot = eqn_d.differentiate().differentiate()(t);
            newpath_s[i]=dfrenet{s,s_dot,s_ddot};
            newpath_d[i]=dfrenet{d,d_dot,d_ddot};
            auto xy =  ioMap.getXYspline(s,d);
            next_x_vals.push_back(xy[0]);
            next_y_vals.push_back(xy[1]);
            t += params::dT;
        }
        returnValue.frenet=interfaces::TrajectoryFrenet{newpath_s,newpath_d};
        return returnValue;
}

/// \brief gerenates frenet for a special case where time =0.0 to generate emergency braking maneuver
/// \see trajectory.h for more details
interfaces::TrajectorySet trajectory::generateTrajectoryFrenet(Target const & i_target, path_planner::Map &ioMap,interfaces::input const & i_input , interfaces::PrevTrajectory const  & i_prev)
{
    interfaces::TrajectorySet returnValue;
    //generate minimum jerk trajectories using a quintic poly solver
    //update state.
    //list of inputs
    auto const & car_s = i_input.currentData.car_s;
    auto const & car_d = i_input.currentData.car_d;
    auto const & car_x = i_input.currentData.car_x;
    auto const & car_y = i_input.currentData.car_y;
    auto const & car_yaw = i_input.currentData.car_yaw;
    auto const & car_speed = i_input.currentData.car_speed;

    //prev values
    auto const &prev_path_x = i_prev.Prev_cart.x_vals;
    auto const &prev_path_y = i_prev.Prev_cart.y_vals;
    auto const &prev_path_s = i_prev.Prev_frenet.s_vals;
    auto const &prev_path_d = i_prev.Prev_frenet.d_vals;  
    //outputs
    auto & next_x_vals = returnValue.cart.x_vals;
    auto & next_y_vals = returnValue.cart.y_vals;
    auto & next_s_vals = returnValue.frenet.s_vals;
    auto & next_d_vals = returnValue.frenet.d_vals;

    vector<dfrenet> newpath_s(params::kMaxPoints,dfrenet{0.0,0.0,0.0});
    vector<dfrenet> newpath_d(params::kMaxPoints,dfrenet{0.0,0.0,0.0});
    
    double target_vel = helper::mph2ms(i_target.velocity);

    double s, s_dot, s_ddot;
    double d, d_dot, d_ddot;
    if (i_prev.prev_reused_xy > 0) 
    {
        for (int i = 0; i < i_prev.prev_reused_xy; i++) 
        {
            newpath_s[i] = prev_path_s[params::kMaxPoints - prev_path_x.size() + i];
            newpath_d[i] = prev_path_d[params::kMaxPoints - prev_path_x.size() + i];
            next_x_vals.push_back(prev_path_x[i]);
            next_y_vals.push_back(prev_path_y[i]);
        }
        // initial conditions for new (s,d) trajectory
        s = newpath_s[i_prev.prev_reused_xy-1].value;
        s_dot = newpath_s[i_prev.prev_reused_xy-1].value_ddot;
        d = newpath_d[i_prev.prev_reused_xy-1].value;
        d_dot = 0.0; 
        d_ddot = 0.0;
    } 
    else 
    {
        s = car_s; 
        s_dot = car_speed;
        d = car_d; 
        d_dot = 0.0;
        d_ddot = 0.0;
    }

  s_ddot = i_target.acc;  //-(comfort acc);

  //double t = 0.0; continuity point reused
  double t = params::dT;
  double prev_s_dot = s_dot;
  for (int i = i_prev.prev_reused_xy;
       i < params::kMaxPoints; 
       i++) 
   {
    // increase/decrease speed till target velocity is reached
    s_dot += s_ddot * params::dT; 
    if ((i_target.acc > 0 && prev_s_dot <= target_vel && s_dot > target_vel) ||
        (i_target.acc < 0 && prev_s_dot >= target_vel && s_dot < target_vel)) 
        {
            s_dot = target_vel;
        }
    s_dot = std::max(std::min(s_dot, 0.9 * params::kMaxHwySpeedLimitms), 0.0);
    s += s_dot * params::dT;

    prev_s_dot = s_dot;

    newpath_s[i] = dfrenet{s, s_dot, s_ddot};
    newpath_d[i] = dfrenet{d, d_dot, d_ddot};

    vector<double> point_xy =  ioMap.getXYspline(s,d);

    next_x_vals.push_back(point_xy[0]);
    next_y_vals.push_back(point_xy[1]);

    t += params::dT;
   }
   returnValue.frenet=interfaces::TrajectoryFrenet{newpath_s,newpath_d};
 return returnValue;
}

/// \brief Jerk Minimalizing Trajectory coefficient generation
/// \see trajectory.h for more details
coefficients trajectory::JMT(const std::vector<double>& i_start, const std::vector<double>& i_end, double T)
{
		coefficients returnValue;
		returnValue.push_back(i_start[0]);
		returnValue.push_back(i_start[1]);
		returnValue.push_back(i_start[2]/2.0);
		// solve for A3 , A4, A5
	// need to use [T]x[A]=[S] and solve for [A] = [T]^-1 x [S]
		double T_2 = T * T;
		double T_3 = T_2 * T;
		double T_4 = T_3 * T;
		// make a computation of T matrix. 
		MatrixXd T_(3, 3);
		T_ << T_3, T_4, T_4* T,
			3 * T_2, 4 * T_3, 5 * T_4,
			6 * T, 12 * T_2, 20 * T_3;
		// check if the matrix is invertible
		if (T_.determinant() != 0)
		{
			MatrixXd S(3, 1);
			S << (i_end[0] - (i_start[0] + i_start[1] * T + 0.5 * i_start[2] * T_2)),
				(i_end[1] - (i_start[1] + i_start[2] * T)),
				(i_end[2] - i_start[2]);
			VectorXd A(3);
			A = T_.inverse() * S;
			returnValue.push_back(A[0]);
			returnValue.push_back(A[1]);
			returnValue.push_back(A[2]);
		}
		else
		{
			m_log<<"solver error. matrix is singular!";
		}
		return returnValue;
	}

}
}