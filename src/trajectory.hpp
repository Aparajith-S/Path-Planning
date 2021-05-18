/// \file
/// \brief
/// \author s.aparajith@live.com
/// \copyright None reserved. MIT license
/// 
/// 
#ifndef TRAJECTORY_H
#define TRAJECTORY_H
#include<vector>
namespace path_planner {
using coefficients = std::vector<double>;

struct trajectory 
{
    coefficients traj_s;
    coefficients traj_d;
    double traj_T;
};
}
#endif