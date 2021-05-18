/// \file
/// \brief
/// \author s.aparajith@live.com
/// \copyright None reserved. MIT license
/// 
/// 
#ifndef CONSTANTS_H
#define CONSTANTS_H
namespace path_planner {
namespace params {

constexpr float kVehicleRadius = 1.5
//Mu and Sigma values
constexpr int N = 10;
const double SIGMA_S[3] = { 10.0, 4.0, 2.0 }; // s, s_dot, s_double_dot
constexpr double SIGMA_D[3] = {1.0, 1.0, 1.0};// d, d_dot, d_double_dot
constexpr double SIGMA_T = 2.0;

//define jerk params
constexpr double kMaxJerk = 1.0; ///>[m/s³];
constexpr double kJerkInOneSec = 2.0;

constexpr double kMaxHwySpeedLimit = 130.0/3.6; //130[km/h] max speed.
//define max and min comfort acceleration params
constexpr double kComfortAcc = 2.5; ///>[m/s²];
constexpr double kComfortDec = -3.5;///>[m/s²];
constexpr double kAccInOneSec = 1.0;

//define max emergency deceleration params
constexpr double kEmergencyDec = -7.0; ///>[m/s/s];

//define step
constexpr float dT = 0.2F;

}
}
    
#endif // !CONSTANTS_H
