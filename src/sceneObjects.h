/// \file
/// \brief
/// \author s.aparajith@live.com
/// \copyright None reserved. MIT license
/// 
/// 
#ifndef SCENE_OBJECTS_H
#define SCENE_OBJECTS_H
#include "interface.h"
#include "trajectory.hpp"
#include "polyfunction.hpp"
#include"constants.h"
#include <limits>
#include <cstdint>
#include<algorithm>
#include<optional>
namespace path_planner {
namespace scene {
struct objectState 
{
    double value;
    double value_dot;
    double value_ddot;
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

struct FrenetState
{
    std::uint8_t id;
    objectState obj_s;
    objectState obj_d;
};

class objects
{
public:
    explicit objects(interfaces::SnsFusionData& i_sensFusionData):
        m_DataRef(i_sensFusionData)
    {}
    virtual ~objects() {}

    std::vector<double> computeFutureState(double time,const FrenetState & objData) const
    {
        std::vector<double> in_state;
        auto s_0 = objData.obj_s.value;
        auto s_1 = objData.obj_s.value_dot;
        auto s_2 = objData.obj_s.value_ddot;

        auto d_0 = objData.obj_d.value;
        auto d_1 = objData.obj_d.value_dot;
        auto d_2 = objData.obj_d.value_ddot;

        in_state.push_back((s_0 + (s_1 * time) + (s_2 * time * time / 2.0)));
        in_state.push_back(s_1 + (s_2 * time));
        in_state.push_back(s_2);
        in_state.push_back(d_0 + (d_1 * time) + (d_2 * time * time / 2.0));
        in_state.push_back(d_1 + d_2 * time);
        in_state.push_back( d_2);
        return in_state;
    }
 
    double calcClosestApproachAnyObject(const trajectory & i_traj)const  
    {
        double closest = std::numeric_limits<double>::max();
        for (const auto& v : m_currentData)
        {
            double dist = calcClosestApproach(i_traj, v);
            if (dist < closest)
            {
                closest = dist;
            }
        }
        return closest;
    }

    std::pair<const FrenetState&,bool> getTargetObject(const std::uint8_t id) const 
    {
        bool found = false;
        auto iter = std::find_if(m_currentData.begin(),
                                 m_currentData.end(),
                                 [id](const FrenetState& match)
        {return (match.id == id); });
        if (iter != m_currentData.end())
        {
            found = true;
            return (std::make_pair(*iter, found));
        }
        else
        {
            return (std::make_pair(*(m_currentData.end() - 1), found));
        }

    }
    
    double calcClosestApproach(const trajectory& i_traj,const FrenetState& objData)const
    {
        double closest = std::numeric_limits<double>::max();
        function f_d; 
        function f_s;
        f_d.constructEquation(i_traj.traj_d);
        f_s.constructEquation(i_traj.traj_s);
        for (int i = 0; i < 100; i++)
        {
            double t = float(i) / 100 * i_traj.traj_T;
            auto cur_s = f_s(t);
            auto cur_d = f_d(t);
            auto state_in = computeFutureState(t, objData);
            double targ_s = state_in[0];
            double targ_d = state_in[3];
            double dist = sqrt(pow((cur_s - targ_s),2) + pow((cur_d - targ_d),2));
            if (dist < closest)
            {
                closest = dist;
            }
        }
        return closest;
    }
    
    object structurizeData(const interfaces::rawObjData& i_rawObj)
    {
        object objData{ i_rawObj[0],
        i_rawObj[1],
        i_rawObj[2],
        i_rawObj[3],
        i_rawObj[4],
        i_rawObj[5],
        i_rawObj[6]};
        return objData;
    }

    // take care to match id and fill values.
    void computeCurrentState(FrenetState& current, const FrenetState& prev)
    {
        current.obj_s.value_dot = (current.obj_s.value-prev.obj_s.value)/params::dT;
        current.obj_s.value_ddot = (current.obj_s.value_dot-prev.obj_s.value_dot)/params::dT;
    }

    void update() 
    {
        m_prevData = m_currentData;
    }

    void refresh()
    {
        m_currentData = std::vector<FrenetState>();
        FrenetState state;
        for (const auto& rawObj : m_DataRef.raw_sensor_fusion)
        {
            auto StData = structurizeData(rawObj);
            state.id = StData.obj_id;
            state.obj_s.value = StData.obj_s;
            state.obj_d.value = StData.obj_d;
            m_currentData.push_back(state);
        }
        computeObjectState();
    }
    
    void computeObjectState()
    {
        //ascending order of ID in the json file. so using this property we update the dot and ddot of s and d. 
        auto curriter = m_currentData.begin();
        auto previter = m_prevData.begin();
        while((previter != m_prevData.end()) && (curriter != m_currentData.end()))
        {
            if (curriter->id== previter->id)
            {
                computeCurrentState(*curriter, *previter);
                ++curriter;
                ++previter;
            }
            else if (curriter->id < previter->id)
            {
                curriter->obj_s.value_ddot = 0.0;
                curriter->obj_s.value_dot = 0.0;
                curriter->obj_d.value_dot = 0.0;
                curriter->obj_d.value_ddot = 0.0;
                ++curriter;
            }
            else 
            {
                previter = m_prevData.erase(previter);
            }
        }

        while (previter != m_prevData.end())
        {
            //remove all old vehicles.
            previter = m_prevData.erase(previter);
        }
    }


private:
    interfaces::SnsFusionData & m_DataRef;
    std::vector<FrenetState> m_currentData;
    std::vector<FrenetState> m_prevData;
};
}
}
#endif // !SCENE_OBJECTS_H
