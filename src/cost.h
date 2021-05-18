/// \file
/// \brief
/// \author s.aparajith@live.com
/// \copyright None reserved. MIT license
/// 
/// 
#ifndef COST_H
#define COST_H
#include<iterator>
#include<vector>
#include<algorithm>
#include"trajectory.hpp"
#include"polyfunction.hpp"
#include"sceneObjects.h"
namespace path_planner{
namespace cost{
/// @brief A function that returns a value between 0 and 1 
/// @details for x in the range[0, infinity] and -1 to 1 for x in the range[-infinity, infinity].
/// @param[in] i_x input.
inline double logistic(const double i_x)
{
    return (2.0 / (1 + exp(-x)) - 1.0);
}

double deltaTcost(const trajectory& i_traj,const double i_T) 
{
    return logistic(fabs(i_traj.traj_T - i_T)/i_T);
}
double sCost(const trajectory& i_traj, 
             const scene::objects & i_objects, 
             const std::uint8_t i_targetObjId,
             const std::vector<double> i_delta,
             const double i_T)
{
    const coefficients &s = i_traj.traj_s;
    const double& T = i_traj.traj_T;
    auto targetObj = i_objects.getTargetObject(i_targetObjId);
    double cost = 0.0;
    if (targetObj.second)
    {
        auto target = i_objects.computeFutureState(i_T, targetObj.first);
        for (int idx = 0; idx < target.size(); idx++)
        {
            target[idx] = target[idx] + i_delta[idx];
        }
        std::vector<double> s_target;
        std::copy(target.begin(), 
                  target.begin() + 3, //copy the first three elements {s,sdot,sddot}
                  std::back_inserter(s_target));
        function f;
        auto funcs = f.differentiateNtimes(s, 2);
        std::vector<double> S;
        for (auto& func : funcs)
        {
            S.push_back(func(T));
        }
        for (int i = 0; i < S.size(); i++)
        {
            double diff = fabs(S[i] - s_target[i]);
            cost += logistic(diff / params::SIGMA_S[i]);
        }
    }
    return cost;
}
double dCost(const trajectory& i_traj,
             const scene::objects& i_objects,
             const std::uint8_t i_targetObjId,
             const std::vector<double> i_delta,
             const double i_T)
{
    const coefficients& d  = i_traj.traj_d;
    const double& T = i_traj.traj_T;
    auto targetObj = i_objects.getTargetObject(i_targetObjId);
    double cost = 0.0;
    if (targetObj.second)
    {
        auto target = i_objects.computeFutureState(i_T, targetObj.first);
        for (int idx = 0; idx < target.size(); idx++)
        {
            target[idx] = target[idx] + i_delta[idx];
        }
        function f;
        f.constructEquation(d);
        auto funcs = f.differentiateNtimes(d, 2);
        std::vector<double> D;
        for (auto& func : funcs)
        {
            D.push_back(func(T));
        }
        std::vector<double> d_target;
        std::copy(target.begin()+3, 
                  target.end(), //copy the last three elements {d,ddot,dddot}
                  std::back_inserter(d_target));
        for (i = 0; i < D.size(); i++)
        {
            double diff = fabs(D[i] - d_target[i]);
            cost += logistic((diff / params::SIGMA_D[i]));
        }
    }
    return cost;
}

double collisionCost(const trajectory& i_traj, const scene::objects& i_objects)
{
    auto closest = i_objects.calcClosestApproachAnyObject(i_traj);
    if (closest < 2 * params::kVehicleRadius)
    {
        return 1.0;
    }
    else return 0.0;
}
double tooCloseForComfortCost(const trajectory& i_traj, const scene::objects& i_objects)
{
    auto closest = i_objects.calcClosestApproachAnyObject(i_traj);
    return logistic((2.0 * params::kVehicleRadius / closest));
}

double efficiencyCost (const trajectory& i_traj,
                             const scene::objects& i_objects,
                             const std::uint8_t i_targetObjId)
{
    const coefficients& s = i_traj.traj_s;
    const double T = i_traj.traj_T;
    function S;
    S.constructEquation(s);
    double avg_v = S(T)/T;
    auto targetObj = i_objects.getTargetObject(i_targetObjId);
    double cost = 0.0;
    if (targetObj.second)
    {
        auto target = i_objects.computeFutureState(i_T, targetObj.first);
        double tgt_s=target[0];
        double tgt_v = tgt_s / T;
        cost = logistic(2 * (tgt_v - avg_v) / avg_v);
    }
    return cost;
}

double maxAccelerationCost(const trajectory& i_traj,double i_time)
{
    const coefficients& s = i_traj.traj_s;
    const coefficients& d = i_traj.traj_d;
    function S;
    S.constructEquation(s);
    S = S.differentiate();
    S = S.differentiate();// this gives s_ddot
    std::vector<float> accs;
    for (int i = 0; i < 100; i++)
    {
        accs.push_back(S((i_time / 100) * i));
    }
    float max_acc = std::max_element(accs.begin(), accs.end());
    float min_acc = std::min_element(accs.begin(), accs.end());
    if (max_acc > params::kComfortAcc || min_acc<params::kComfortDec)
    {
        return 1.0;
    }
    else
    {
        return 0.0;
    }
}
double maxJerkCost(const trajectory& i_traj, double i_time)
{
    const coefficients& s = i_traj.traj_s;
    const coefficients& d = i_traj.traj_d;
    function S;
    S.constructEquation(s);
    S = S.differentiate();
    S = S.differentiate();
    S = S.differentiate();// this gives jerk. d�s/dt�
    std::vector<double> jerks;
    for (int i = 0; i < 100; i++)
    {
        jerks.push_back(S((i_time / 100) * i));
    }
    float max_jerk = std::max_element(accs.begin(),
                                      accs.end(), 
                                      [](const float a, const float b) 
    {return abs(a) < abs(b); });
    if (abs(max_jerk) > params::kMaxJerk)
    {
        return 1.0;
    }
    else
    {
        return 0.0;
    }
}

double totalJerkCost(const trajectory& i_traj, double i_time)
{
    const coefficients& s = i_traj.traj_s;
    const coefficients& d = i_traj.traj_d;
    function S;
    S.constructEquation(s);
    S = S.differentiate();
    S = S.differentiate();
    S = S.differentiate();// this gives jerk. d�s/dt�
    double total_jerk=0.0;
    double dt = i_time / 100.0;
    for (int i = 0; i < 100; i++)
    {
        double j = S(dt * i);
        total_jerk += abs(j * dt);
    }
    double jerkps = total_jerk / i_time;
    return logistic(jerkps / params::kJerkInOneSec);
}

double totalAccelerationCost(const trajectory& i_traj, double i_time)
{
    const coefficients& s = i_traj.traj_s;
    const coefficients& d = i_traj.traj_d;
    function S;
    S.constructEquation(s);
    S = S.differentiate();
    S = S.differentiate();
    double total_acc = 0.0;
    double dt = i_time / 100.0;
    for (int i = 0; i < 100; i++)
    {
        double acc = S(dt * i);
        total_acc += abs(acc * dt);
    }
    double accps = total_acc / i_time;
    return logistic(accps / params::kAccInOneSec);
}

//this encorporates the Rechtsfahrgebot and Rechtsueberholverbot. 
double legalCost() 
{
    //TODO based on legal allowed driving behavior
}
}
}
#endif // !COST_H
