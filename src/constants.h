/// \file
/// \brief
/// \author s.aparajith@live.com
/// \copyright None reserved. MIT license
/// 
/// 
#ifndef CONSTANTS_H
#define CONSTANTS_H
#include <cstdint>
namespace params {
constexpr bool kUseJMT = true;
constexpr double kMapEnd = 6945.554;
constexpr std::uint8_t kMaxNrLanes = 3U; 
constexpr std::uint8_t kLeft = 0U;
constexpr std::uint8_t kCenter = 1U;
constexpr std::uint8_t kRight = 2U;
constexpr float kVehicleRadius = 1.5;
constexpr int N = 10;
constexpr int kMaxPoints=50;
constexpr int kMaxPointsReused=5;

constexpr double kWeightEfficiency = 1.0;
constexpr double kWeightLegal = 1.0;
constexpr double kWeightComfort = 1.0;
constexpr double kWeightCollision = 10000.0;
//define jerk params
constexpr double kMaxJerk = 10.0; ///>[m/s�];
constexpr double kJerkInOneSec = 2.0;

constexpr double kMaxHwySpeedLimit = 49.0; //49[mi/h] max speed.
constexpr double kMaxHwySpeedLimitms = 22.0; //49[mi/h] max speed.
//define max and min comfort acceleration params
constexpr double kComfortAcc = 10; ///>[m/s�];
constexpr double kComfortDec = 10;///>[m/s�];
constexpr double kMaxAcc = 10.0; ///>[m/s�];
constexpr double kTotalAcc = 10.0; ///>[m/s�];
constexpr double kAccInOneSec = 1.0;
//define max emergency deceleration params
constexpr double kEmergencyDec = 7.0; ///>[m/s/s];

//define step
constexpr float dT = 0.02F;
constexpr double kMaxSpeedIncrement = kComfortAcc*dT;
constexpr double kLaneWidth=4.0;

constexpr double kPredictionHorizon = 70.0;
constexpr double kSafeProximity = 25.0;
constexpr double kSafeProximityLC = 10.0;

// object properties
constexpr double kLengthBoundingBox = 4;
constexpr double kWidthBoundingBox = 2;
}
    
#endif // !CONSTANTS_H
