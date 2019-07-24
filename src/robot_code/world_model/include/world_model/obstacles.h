#ifndef NUBOT_OBSTACLES_H_
#define NUBOT_OBSTACLES_H_

#include "core.hpp"

namespace nubot {
const int MAX_OBSNUMBER_CONST = 10;
class ObstacleObject{

public:
    ObstacleObject(DPoint2s _loc=DPoint2s(-10000,-10000),PPoint _polar=PPoint(Angle(0),10000));
    ObstacleObject(const ObstacleObject & _info);
    const ObstacleObject & operator=(const ObstacleObject & _info)
    {
        obstacle_loc_   = _info.obstacle_loc_;
        obstacle_polar_ = _info.obstacle_polar_;
        return *this;
    }
    void clear();
    void setLocation(DPoint2s _loc);
    void setPolarLocation(PPoint _polar);
    DPoint2s getLocation();
    PPoint getPolarLocation();
private:
    DPoint2s obstacle_loc_;
    PPoint   obstacle_polar_;
};

class Obstacles{
    struct obstacles_sort
    {
        double distance_;
        DPoint obstacle_;
    };

public:
    Obstacles(void);
    ~Obstacles(void);
    void setOmniObstacles(std::vector< ObstacleObject > & _obstacles,int robot_id =1);
    void getOmniObstacles(std::vector< ObstacleObject > & _omni_obstacles,int robot_id=1);
    void setKinectObstacles(std::vector< ObstacleObject > & _obstacles, int robot_id =1);
    void getKinectObstacles(std::vector< ObstacleObject > & kinect_obstacles,int robot_id =1);
    void clearOmniObstacles(int robot_id=1);
    void clearKinectObstacles(int robot_id=1);
    void getFuseObsTracker(std::vector<DPoint> & _obs_tracker);
    void getSelfObsTracker(std::vector<DPoint> & _obs_tracker);
    void MultiTargetTrackKalmanFilter();
    void update();
    void setAgentID(int AgentID);
    void setRobotInfo(DPoint robot_pos, bool isValid, int robot_id =1);
public:
    std::vector < std::vector< ObstacleObject > > omni_obstacles_;
    std::vector< ObstacleObject >  kinect_obstacles_;
    std::vector< std::vector< obs_info > > obs_measure_;
    std::vector< DPoint > fuse_obs_;
    std::vector< DPoint > self_obs_;
    std::vector< DPoint > robot_pos_;
    std::vector< bool >   robot_valid_;
    std::vector< obstacles_sort> sort_info_;
    int AgentID_;
};

}

#endif // OBSTACLEINFO_H
