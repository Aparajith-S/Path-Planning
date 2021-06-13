/// \file sceneObjects.cpp
/// \brief scene object class with prediction and ttc/ ttd calculations
/// \author s.aparajith@live.com
/// \date 30.05.2021
/// \copyright None reserved. MIT license
#include "helpers.h"
#include "polyfunction.h"
#include "constants.h"
#include "sceneObjects.h"
#include "interface.h"
#include "logging.h"
#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>
#include <iostream>
#include <cassert>
#include<fstream>
namespace path_planner {
namespace scene {
using interfaces::chosenTrajectory;
using std::cout;
using std::endl;

/// @brief predicts the future trajectories of the vehicles surrounding the ego.
/// @see sceneObjects.h for more details
void objects::predict(int i_predictionHorizon)
  {
        m_predictions.clear();
        constexpr std::uint8_t kInvalidObjectId=255U;
        //compute prediction trajectories for the frontal objects
        objectKinematics k;
        for(int i = 0;i<m_FrontRelevantObjects.size();i++)
        {
            if(m_FrontRelevantObjects[i]!=kInvalidObjectId)
            {
                double x = m_DataRef.raw_sensor_fusion[m_FrontRelevantObjects[i]][1];
                double y = m_DataRef.raw_sensor_fusion[m_FrontRelevantObjects[i]][2];
                double vx = m_DataRef.raw_sensor_fusion[m_FrontRelevantObjects[i]][3];
                double vy = m_DataRef.raw_sensor_fusion[m_FrontRelevantObjects[i]][4];
                
                std::vector<vector2d> coords;
                for(int j=0;j<i_predictionHorizon;j++)
                {vector2d vec;
                    vec.x = x + vx*params::dT;
                    vec.y = y + vy*params::dT;
                    coords.push_back(vec);
                }
                m_predictions[m_FrontRelevantObjects[i]]=coords;
            }
        }
        //compute prediction trajectories for the rear objects
        for(int i = 0;i<m_RearRelevantObjects.size();i++)
        {
            if(m_RearRelevantObjects[i]!=kInvalidObjectId)
            {
                double x = m_DataRef.raw_sensor_fusion[m_RearRelevantObjects[i]][1];
                double y = m_DataRef.raw_sensor_fusion[m_RearRelevantObjects[i]][2];
                double vx = m_DataRef.raw_sensor_fusion[m_RearRelevantObjects[i]][3];
                double vy = m_DataRef.raw_sensor_fusion[m_RearRelevantObjects[i]][4];
                std::vector<vector2d> coords;
                for(int j=0;j<i_predictionHorizon;j++)
                {vector2d vec;
                    vec.x = x + vx*params::dT;
                    vec.y = y + vy*params::dT;
                    coords.push_back(vec);
                }
                m_predictions[m_RearRelevantObjects[i]]=coords;
            }
        }
    }

/// @brief computes the safe proximity given the \p i_frontVel and \p i_rearVel velocity 
/// @see sceneObjects.h for more details
double objects::computeSafeProximity(double i_rearVel, double i_frontVel, double time) 
{
  double safeRearProx = params::kSafeProximityLC;
  // this is a problem - prevent rear ending into the Ego.
  if (i_rearVel > i_frontVel) 
  {
      double ttd = (i_rearVel - i_frontVel) / m_EgoAllowedDecel + time;
      safeRearProx = i_rearVel * ttd + 1.5 * params::kLengthBoundingBox;
  }
  safeRearProx = std::max(safeRearProx, params::kSafeProximityLC);
  return safeRearProx;
}

/// @brief computes proximities based on \p i_data
/// @see sceneObjects.h for more details
void objects::computeSafeProximities(interfaces::EgoData const & i_data)
{
  m_EgoSpeed = helper::mph2ms(i_data.car_speed); 

  // slightly conservative as it will relate to safety distance
  m_EgoAllowedDecel = 0.8 * params::kEmergencyDec;
  static_assert(params::kEmergencyDec>0.0,"divide by 0. kEmergencyDec should not be 0");
  m_TStop = m_EgoSpeed / m_EgoAllowedDecel;
  m_velFrontVeh = getObjectVelocity(m_FrontRelevantObjects[i_data.car_lane], params::kMaxHwySpeedLimitms);
  m_distFrontVeh = m_DistfrontMin[i_data.car_lane];
  if (m_EgoSpeed > m_velFrontVeh) 
  {
    m_TTC = m_distFrontVeh / (m_EgoSpeed - m_velFrontVeh);
    m_TTD = (m_EgoSpeed - m_velFrontVeh) / m_EgoAllowedDecel;
    m_safe_proximity = m_EgoSpeed * m_TTD + 1.75 * params::kLengthBoundingBox;
  } else 
  {
    m_TTC = std::numeric_limits<double>::max();
    m_TTD = 0.0;
    m_safe_proximity = 1.75 * params::kLengthBoundingBox; 
  }
  m_emergency_proximity = m_EgoSpeed * m_TStop + 2 * params::kLengthBoundingBox;
  
  for (int i = 0; i < params::kMaxNrLanes; i++) {
    m_SpeedFront[i] = getObjectVelocity(m_FrontRelevantObjects[i], params::kMaxHwySpeedLimitms);
    m_frontLCSafeProximity[i] = computeSafeProximity(m_EgoSpeed, m_SpeedFront[i], 0.0);

    m_SpeedRear[i] = getObjectVelocity(m_RearRelevantObjects[i], 0);
    m_rearLCSafeProximity[i] = computeSafeProximity(m_SpeedRear[i], m_EgoSpeed, 2.0); 
  }

}

/// @brief computes the velocity of the object at the \p i_indx in the sensor fusion array
/// @see sceneObjects.h for more details
double objects::getObjectVelocity(std::uint8_t idx, double default_vel)
{
  double vx;
  double vy;
  double vel=default_vel;
  if (idx >= 0 && idx < m_DataRef.raw_sensor_fusion.size()) {
    vx = m_DataRef.raw_sensor_fusion[idx][3];
    vy = m_DataRef.raw_sensor_fusion[idx][4];
    vel = sqrt(vx*vx+vy*vy);
  }
  return vel;
}

/// @brief sets the safe lane speed and free space in the \p i_lane lane
/// @see sceneObjects.h for more details
void objects::setLaneParams(interfaces::EgoData const &car)
{
  int car_lane = car.car_lane;
  for (size_t i = 0; i < m_FrontRelevantObjects.size(); i++) 
  {
    int lane = i;
    if (m_FrontRelevantObjects[i] !=255U) 
    { // a car in front of us
      //if (lane != car_lane && (m_RearRelevantObjectsdmin_[i] <= 10 || m_FrontRelevantObjectsdmin_[i] <= 10)) {
      if (lane != car_lane && (m_DistrearMin[i] <= m_rearLCSafeProximity[i] || 
          m_DistfrontMin[i] <= m_frontLCSafeProximity[i])) 
      {
        m_laneParam.m_laneSpeed[i] = 0;
        m_laneParam.m_laneFreeSpace[i] = 0; // too dangerous
      } 
      else 
      {
        double vx = m_DataRef.raw_sensor_fusion[m_FrontRelevantObjects[i]][3];
        double vy = m_DataRef.raw_sensor_fusion[m_FrontRelevantObjects[i]][4];
        m_laneParam.m_laneSpeed[i] = std::min(helper::ms2mph(sqrt(vx*vx+vy*vy)),params::kMaxHwySpeedLimit);
        m_laneParam.m_laneFreeSpace[i] = m_DistfrontMin[i];
      }
    } 
    else 
    {  // if nobody in front of us
      //if (lane != car_lane && m_RearRelevantObjectsdmin_[i] <= 10) {
      if (lane != car_lane && m_DistrearMin[i] <= m_rearLCSafeProximity[i]) 
      {
        m_laneParam.m_laneSpeed[i] = 0;
        m_laneParam.m_laneFreeSpace[i] = 0; // too dangerous
      } else {
        m_laneParam.m_laneSpeed[i] = params::kMaxHwySpeedLimit;
        m_laneParam.m_laneFreeSpace[i] = params::kPredictionHorizon;
      }
    }
    //std::cout << "Predictions::lane_speed_[" << i << "]=" << m_laneParam.m_laneSpeed[i] << std::endl;
  }
}

/// @brief gets the safe lane speed in the \p i_lane lane
/// @see sceneObjects.h for more details
double objects::getSafeLaneSpeed(std::uint8_t lane) const {
  if (lane >= 0 && lane < 3) {
    return m_laneParam.m_laneSpeed[lane];
  } 
  else 
  {
    return 0.0;
  }
}

/// @brief gets the free space in the \p i_lane lane
/// @see sceneObjects.h for more details
double objects::getLaneFreeSpace(std::uint8_t lane) const {
  if (lane >= 0 && lane < 3) {
    return m_laneParam.m_laneFreeSpace[lane];
  } else {
    return 0.0;
  }
}

/// @brief refreshes internal data 
/// @see sceneObjects.h for more details    
void objects::refreshData(interfaces::SnsFusionData const & snsdata,interfaces::EgoData const & i_egoData )
    {   m_cyclicCounter++;
        m_DataRef= snsdata;
        double maxHorizon = i_egoData.car_s + params::kPredictionHorizon;
        double minHorizon = i_egoData.car_s - params::kPredictionHorizon;
        m_RearRelevantObjects=std::vector<uint8_t>(3U,255U);
        m_FrontRelevantObjects=std::vector<uint8_t>(3U,255U);
        std::vector<double>min_front(3,std::numeric_limits<double>::max());
        std::vector<double>min_rear(3,std::numeric_limits<double>::max());
        // the shift horizon will move the prediction horizon of the ego with 
        // objects in the event of the horizons containing the end of map coordinates. 
        double shiftHorizon=0.0;
        if (minHorizon < 0) 
        { 
            shiftHorizon = -minHorizon;
            minHorizon=0.0;
        } 
        else if (maxHorizon > params::kMapEnd) 
        {
            shiftHorizon = params::kMapEnd - maxHorizon;
            maxHorizon=params::kMapEnd;
        }
        double car_s = i_egoData.car_s+shiftHorizon;
        uint8_t iter=0;
        for (const auto& rawObj : m_DataRef.raw_sensor_fusion)
        {
            object StData = structurizeData(rawObj);
            double lane = helper::getLane(StData.obj_d);
         //ignore invalid objects
            if((lane >= 0 )&&(lane <= 2))
            {  	double shifted_obj_s = StData.obj_s+shiftHorizon;
                if((shifted_obj_s >=minHorizon) && 
                   (shifted_obj_s<=maxHorizon))
                {
                    double distance = fabs(shifted_obj_s - car_s);
                    if(shifted_obj_s < car_s)//rear
                    {
                        if(distance<min_rear.at(lane))
                        {
                            m_RearRelevantObjects[lane]=iter;
                            min_rear[lane]=distance;
                        }
                    }else //front
                    {
                         if(distance<min_front.at(lane))
                        {
                          m_FrontRelevantObjects[lane]=iter;
                          min_front[lane]=distance;
                        }
                    }
                }
            }
            iter++;
        }
        //save to cumpute TTC and proximities later
        m_DistfrontMin = min_front;
        m_DistrearMin = min_rear;
    }

/// @brief structurizes the data for easier use
/// @see sceneObjects.h for more details
object objects::structurizeData(const interfaces::rawObjData& i_rawObj)
    {
        object objData{ static_cast<std::uint8_t>(i_rawObj[0]),
        i_rawObj[1],
        i_rawObj[2],
        i_rawObj[3],
        i_rawObj[4],
        i_rawObj[5],
        i_rawObj[6]};
        return objData;
    }

/// @brief get private prediction data 
/// @see sceneObjects.h for more details
std::map<std::uint8_t,std::vector<vector2d>> objects::getPredictions() const
{
  return m_predictions;
}

/// @brief log to file
/// @see sceneObjects.h for more details
void objects::log_objs_to_file(interfaces::input const& i_input,int cyclecount)
    {
        std::ofstream fout,fout2,fout3;
      	if(cyclecount==0)
        {
            fout.open("ObjOpLog.csv", std::ofstream::out);
            fout2.open("ObjIpLog.csv", std::ofstream::out);
            fout2<<"cycle,id,x,y,vx,vy,s,d,Ego_s,Ego_d,Ego_yaw,Ego_speed"<<std::endl;
       	    fout<<"cycle,left_front,center_front,right_front,left_rear,center_rear,right_rear,lane_speed_left,lane_speed_center,lane_speed_right"<<std::endl;
        }
        else
        {
            fout.open("ObjOpLog.csv", std::ofstream::out|std::ofstream::app);
            fout2.open("ObjIpLog.csv", std::ofstream::out|std::ofstream::app);
        }
        for(const auto & in:i_input.snsFusion.raw_sensor_fusion)
        {
          fout2<<cyclecount<<','<<in[0]<<","<<
          in[1]<<","<<in[2]<<","<<in[3]<<","<<in[4]<<","
          <<in[5]<<","<<in[6]<<","<<i_input.currentData.car_s<<","<<i_input.currentData.car_d
          <<","<<i_input.currentData.car_yaw<<","<<i_input.currentData.car_speed<<std::endl;
        }       
      	fout<<cyclecount<<','<<(int)m_FrontRelevantObjects[0]<<','<<(int)m_FrontRelevantObjects[1]<<','<<(int)m_FrontRelevantObjects[2]<<','<<
        (int)m_RearRelevantObjects[0]<<','<<(int)m_RearRelevantObjects[1]<<','<<(int)m_RearRelevantObjects[2]<<','<<getSafeLaneSpeed(0)<<','<<
        getSafeLaneSpeed(1)<<','<<getSafeLaneSpeed(2)<<std::endl;
      fout.close();
      fout2.close();
    }
}
}