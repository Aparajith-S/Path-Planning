/// \file sceneObjects.h
/// \brief scene object class with prediction and ttc/ ttd calculations
/// \author s.aparajith@live.com
/// \date 30.05.2021
/// \copyright None reserved. MIT license
#ifndef SCENE_OBJECTS_H
#define SCENE_OBJECTS_H
#include "interface.h"
#include "types.h"
#include"constants.h"
#include <cstdint>
#include <map>
#include <vector>
namespace path_planner {
namespace scene {

class objects
{
public:
    explicit objects(interfaces::SnsFusionData const & i_sensFusionData):
        m_DataRef(i_sensFusionData),
        m_FrontRelevantObjects(3U,255U),
        m_RearRelevantObjects(3U,255U),
        m_predictions(),
        m_distFrontVeh(0),
        m_velFrontVeh(0),
        m_laneParam(0.0,0.0),
        m_ProximityFront(std::numeric_limits<double>::max()),
        m_TTC(std::numeric_limits<double>::max()),
        m_TTD(0.0),
        m_TStop(0.0),
        m_safe_proximity(params::kSafeProximity),
        m_emergency_proximity(),
        m_DistfrontMin(3U,std::numeric_limits<double>::max()),
        m_DistrearMin(3U,std::numeric_limits<double>::max()),
        m_SpeedFront(3U,params::kMaxHwySpeedLimit),
        m_SpeedRear(3U,params::kMaxHwySpeedLimit),
        m_frontLCSafeProximity(3U,params::kSafeProximityLC),
        m_rearLCSafeProximity(3U,params::kSafeProximityLC),
        m_cyclicCounter(-1)
    {}
    virtual ~objects() {}

    /// @brief get private prediction data 
    /// @return map data containing prediction coordinates
    std::map<std::uint8_t,std::vector<vector2d>> getPredictions()const;
    
    /// @brief gets safe proximity on the front in the ego lane.
    /// @details updates the TTC and TTD as well
    /// @return safe proximity in ego lane in [m].
    double getSafeProximity()const{return m_safe_proximity;};
    
    /// @brief gets the bumper to bumper distance to the close front vehicle in the given lane number
    /// @param [in] lane lane id {0: left, 1: center, 2: right}
    /// @return distance to the front closest vehicle in the requested lane
    double getSafeDistanceFront(uint8_t lane){return m_DistfrontMin[lane];}
    
    /// @brief gets the bumper to bumper distance to the close rear vehicle in the given lane number
    /// @param [in] lane lane id {0: left, 1: center, 2: right}
    /// @return distance to the rear closest vehicle in the requested lane
    double getSafeDistanceRear(uint8_t lane){return m_DistrearMin[lane];}
    
    /// @brief gets Emergency Proximity data
    /// @return emergency distance computation in meters
    double getEmergencyProximity()const{return m_emergency_proximity;};
    
    /// @brief computes proximities based on \p i_data
    /// @param [in] i_data : input vehicle data
    void computeSafeProximities(interfaces::EgoData const & i_data);
    
    /// @brief predicts the future trajectories of the vehicles surrounding the ego.
    /// @param [in] i_predictionHorizon : how many points to predict in the future?
    void predict(int i_predictionHorizon=params::kMaxPoints);
    
    /// @brief refreshes internal data 
    /// @details excludes vehicles outside the prediction horizon read from \p snsdata
    /// @param [in] snsdata : sensor fusion data 
    /// @param [in] i_egoData : input car data
    void refreshData(interfaces::SnsFusionData const & snsdata,interfaces::EgoData const & i_egoData );
    
    /// @brief gets the safe lane speed in the \p i_lane lane
    /// @param [in] i_lane lane number
    /// @return safe lane speed in miles/h
    double getSafeLaneSpeed(std::uint8_t i_lane) const;
    
    /// @brief gets the free space in the \p i_lane lane
    /// @param [in] i_lane lane number
    /// @return gets free space in meters
    double getLaneFreeSpace(std::uint8_t i_lane) const;
    
    /// @brief sets the safe lane speed and free space in the \p i_lane lane
    /// @param [in] i_lane lane number
    void setLaneParams(interfaces::EgoData const &i_data);
    
    /// @brief log to file
    /// @details logs the input object and processed objects
    /// @param [in] i_input : input interface to the path planner
    /// @param [in] cyclecount : cycle count of the planner
    /// @return
    void log_objs_to_file(interfaces::input const& i_input,int cyclecount);

private:
    /// @brief computes the safe proximity given the \p i_frontVel and \p i_rearVel velocity 
    /// @details uses the \p i_time as an extra reaction time and computes the 
    /// @param [in] i_rearVel : rear vehicle velocity in m/s
    /// @param [in] i_frontVel : front vehicle velocity in m/s
    /// @param [in] i_time : extra reaction time in s
    /// @return proximity in meters
    double computeSafeProximity(double i_rearVel, double i_frontVel, double i_time);
    
    /// @brief computes the velocity of the object at the \p i_indx in the sensor fusion array
    /// @param [in] i_indx : index of the record in sensor fusion array
    /// @return object velocity
    double getObjectVelocity(std::uint8_t i_indx, double default_vel);
    
    /// @brief structurizes the data for easier use
    /// @param [in] i_rawObj the raw sensor fusion data
    /// @return object : data structure structurizing the object information
    object structurizeData(const interfaces::rawObjData& i_rawObj);
    interfaces::SnsFusionData m_DataRef;
    std::vector<std::uint8_t> m_FrontRelevantObjects;
    std::vector<std::uint8_t> m_RearRelevantObjects;
    std::map<std::uint8_t,std::vector<vector2d>> m_predictions;
    double m_distFrontVeh;
    double m_velFrontVeh;
    laneParameters m_laneParam;
    //predict TTC and TTD params
    double m_EgoSpeed;
    double m_EgoAllowedDecel;
    //proximities and time
    double m_ProximityFront;
    double m_TTC; // time to collision
    double m_TTD; // time to decelerate
    double m_TStop;
    double m_safe_proximity;
    double m_emergency_proximity;
    std::vector<double> m_DistfrontMin;
    std::vector<double> m_DistrearMin;
    std::vector<double> m_SpeedFront;
    std::vector<double> m_SpeedRear;
    std::vector<double> m_frontLCSafeProximity;
    std::vector<double> m_rearLCSafeProximity;
    int m_cyclicCounter;
};
}
}
#endif // !SCENE_OBJECTS_H
