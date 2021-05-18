/// \file
/// \brief
/// \author s.aparajith@live.com
/// \copyright None reserved. MIT license
/// 
/// 
#include<vector>
#include<iostream>
#include <random> // Need this for sampling from distributions
#include "path_planner.h"
#include "interface.h"
#include "trajectory.hpp"
#include "constants.h"
using std::normal_distribution;
namespace path_planner {
using Eigen::MatrixXd;
using Eigen::VectorXd;
	PathPlanner::PathPlanner() :
		m_state(State::INIT),
		m_initialized(false),
		m_pos_x(0.0),
		m_pos_y(0.0),
		m_d(0.0),
		m_s(0.0),
		m_yaw(0.0)
	{}
	void PathPlanner::init(interfaces::input const& i_input)
	{
		if (i_input.prevOp.previous_path_x.size() == 0) 
		{
			m_pos_x = i_input.currentData.car_x;
			m_pos_y = i_input.currentData.car_y;
			m_d_start.value = i_input.currentData.car_d;
			m_s_start.value = i_input.currentData.car_s;
			m_yaw = deg2rad(i_input.currentData.car_yaw);
			m_speed = i_input.currentData.car_speed;
		}
	}
	void PathPlanner::process(interfaces::input const& i_input, interfaces::output& o_trajectory)
	{
		//compute current parameters of the vehicle based on previous values 
		//get target speed for that lane.
		
		//get obstacles and their velocities within +/- the horizon.
		
		//set goal for all possible transitions
		//compute costs for possible transistions
		//generate minimum jerk trajectories using a quintic poly solver
		//update state.
		if (m_initialized)
		{
			if (i_input.prevOp.previous_path_x.size() != 0)
			{
				m_pos_x = i_input.prevOp.previous_path_x[path_size - 1];
				m_pos_y = i_input.prevOp.previous_path_y[path_size - 1];
				double pos_x2 = i_input.prevOp.previous_path_x[path_size - 2];
				double pos_y2 = i_input.prevOp.previous_path_y[path_size - 2];
				m_yaw = atan2(m_pos_y - pos_y2, m_pos_x - pos_x2);
				auto result = getFrenet(i_input.prevOp.previous_path_x[path_size - 1],
										i_input.prevOp.previous_path_y[path_size - 1],
										m_yaw,
										i_input.snsFusion.map_waypoints_x,
										i_input.snsFusion.map_waypoints_y);
				m_s_start.value = result[0];
				m_d_start.value = result[1];

			}


		}
	}
	
	coefficients PathPlanner::JMT(const std::vector<double>& i_start, const std::vector<double>& i_end, double T)
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
			std::cout<<"solver error. matrix is singular!";
		}
		return returnValue;
	}
	
	std::pair< std::vector<double>, std::vector<double>> PathPlanner::perturbedGoal(const std::vector<double>& i_goal_s, const std::vector<double>& i_goal_d)
	{
		std::vector<double> new_goal_s;
		std::vector<double> new_goal_d;
		std::default_random_engine gen(42);
		for (int i = 0; i < i_goal_s.size(); i++)
		{
			normal_distribution<double> dist_s(i_goal_s[i], params::SIGMA_S[i]);
			new_goal_s.push_back(dist_s(gen));
			normal_distribution<double> dist_d(i_goal_d[i], params::SIGMA_D[i]);
			new_goal_d.push_back(dist_d(gen));
		}

		return std::make_pair(new_goal_s, new_goal_d);
	}


}