/// \file types.h
/// \brief project wide types
/// \author s.aparajith@live.com
/// \date 08.06.2021
/// \copyright None reserved. MIT license
#ifndef TYPES_H
#define TYPES_H
#include "geometry.h"
#include <cstdint>
#include <cmath>
#include <vector>
#include <limits>
namespace path_planner{

constexpr std::uint8_t kMaxHorizon = 50U; // distance Horizon for planning

struct Target
{
    Target():lane(0),
    velocity(0.0),
    time(0.0),
    acc(0.0),
    cost(0.0),
    danger(false){}
    ~Target(){}
    uint8_t id;
    double lane;
    double velocity;
    double time;
    double acc;
    double cost;
    double danger;
};

namespace scene {
struct laneParameters
{
    std::vector<double> m_laneSpeed;
    std::vector<double> m_laneFreeSpace;
    laneParameters(double def1,double def2):
    m_laneSpeed(3,def1),
    m_laneFreeSpace(3,def2){}
};    
struct objectKinematics 
{
    std::uint8_t id; // assuming <=255 objects   
    double x;
    double y;
    double vx;
    double vy;
    double speed;
};

struct PredictionKinematics
{
    vector2d xy;
};

struct object
{
    std::uint8_t obj_id; // assuming <=255 objects
    double obj_x;
    double obj_y;
    double obj_Vx;
    double obj_Vy;
    double obj_s;
    double obj_d;
};


} // namespace scene 


namespace trajectory
{



} // namespace trajectory 

namespace cost
{

}// namespace cost
}

#endif