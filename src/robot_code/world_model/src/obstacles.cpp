#include "world_model/obstacles.h"
#include "world_model/MTTracker.h"

using namespace nubot;
ObstacleObject::ObstacleObject(DPoint2s _loc,PPoint _polar)
{
    obstacle_loc_  = _loc;
    obstacle_polar_= _polar;
}
ObstacleObject::ObstacleObject(const ObstacleObject & _info)
{
    obstacle_loc_   = _info.obstacle_loc_;
    obstacle_polar_ = _info.obstacle_polar_;
}
void
ObstacleObject::setLocation(DPoint2s  _loc) { obstacle_loc_=_loc; }
void
ObstacleObject::clear()
{
    obstacle_loc_   = DPoint2s(-10000,-10000);
    obstacle_polar_ = PPoint(Angle(0),10000);
}
void
ObstacleObject::setPolarLocation(PPoint _polar) { obstacle_polar_=_polar;}
DPoint2s
ObstacleObject::getLocation() { return obstacle_loc_;}
PPoint
ObstacleObject::getPolarLocation() { return obstacle_polar_;}

Obstacles::Obstacles(void)
{
    omni_obstacles_.resize(OUR_TEAM);
    obs_measure_.resize(OUR_TEAM);
    robot_pos_.resize(OUR_TEAM);
    robot_valid_.resize(OUR_TEAM);
    for(int i = 0 ; i < OUR_TEAM ; i++)
        omni_obstacles_[i].clear();
    self_obs_.clear();
    fuse_obs_.clear();
}

Obstacles::~Obstacles(void)
{
    omni_obstacles_.clear();
    self_obs_.clear();
    fuse_obs_.clear();
    obs_measure_.clear();
}
void Obstacles::setAgentID(int AgentID){
    AgentID_ = AgentID;
}
void
Obstacles::setRobotInfo(DPoint robot_pos, bool isValid, int robot_id)
{
    robot_pos_[robot_id-1]   = robot_pos;
    robot_valid_[robot_id-1] = isValid;
}

void
Obstacles::setOmniObstacles(std::vector< ObstacleObject > & _obstacles, int robot_id)
{
    omni_obstacles_[robot_id-1].clear();
    int nums_obstacles= _obstacles.size();
    for(std::size_t i =0 ;i < nums_obstacles;i++)
    {
        ObstacleObject & obs = _obstacles[i];
        /// PolarLocation()的初始值为10000,如果此时仍是10000,则表明该障碍物无效
        if(obs.getPolarLocation().radius_ == 10000)
             continue;
        omni_obstacles_[robot_id-1].push_back(_obstacles[i]);
    }
}

/// \brief Kinect的障碍物信息并未做融合,因为RTDB没有将Kinect的信息共享,所以无法像全向一样融合,因此这里Kinect函数中的robot_id值是无用的
void
Obstacles::setKinectObstacles(std::vector< ObstacleObject > & _obstacles, int robot_id)
{
    kinect_obstacles_.clear();
    int nums_obstacles= _obstacles.size();
    for(std::size_t i =0 ;i < nums_obstacles;i++)
    {
        ObstacleObject & obs = _obstacles[i];
//        if(obs.getPolarLocation().radius_ == 10000)
//             continue;
        kinect_obstacles_.push_back(_obstacles[i]);
    }
}
void
Obstacles::clearOmniObstacles(int robot_id){
  omni_obstacles_[robot_id-1].clear();
}
void
Obstacles::clearKinectObstacles(int robot_id){
  kinect_obstacles_.clear();

}
void
Obstacles::getFuseObsTracker(std::vector<DPoint> & _obs_tracker){
    _obs_tracker = fuse_obs_;
}
void
Obstacles::getSelfObsTracker(std::vector<DPoint> & _obs_tracker){
    _obs_tracker = self_obs_;
}

void
Obstacles::getOmniObstacles(std::vector< ObstacleObject > & _omni_obstacles,int robot_id){
    _omni_obstacles=omni_obstacles_[robot_id-1];
}

void
Obstacles::getKinectObstacles(std::vector< ObstacleObject > & kinect_obstacles,int robot_id){
    kinect_obstacles=kinect_obstacles_;
}

void
Obstacles::update(){
    MultiTargetTrackKalmanFilter();
}
void
Obstacles::MultiTargetTrackKalmanFilter()
{
    static MTTracker self_tracker;
    static MTTracker tracker;
    Teammates teammates;
    for(size_t i=0; i < OUR_TEAM; i++)
    {
        if(robot_valid_[i])
            teammates[i+1] = robot_pos_[i];
    }
    tracker.TracksPrediction();
    self_tracker.TracksPrediction();

    BOOST_FOREACH(Teammate &tm, teammates)
    {
        int    robot_id  = tm.first;
        DPoint robot_pos = tm.second;
        std::vector<ObstacleObject> & obstacle_tmp = omni_obstacles_[robot_id-1];
        std::vector<obs_info> & measure_datas      = obs_measure_[robot_id -1];
        measure_datas.clear();
        obs_info mt;
        int length = obstacle_tmp.size();
        for(size_t j=0; j < length ; j++)
        {
             mt.polar_pt  = obstacle_tmp[j].getPolarLocation();
             mt.world_pt  = obstacle_tmp[j].getLocation();
             Filter::CalcInformationMatrix(mt);
             measure_datas.push_back(mt);
       }
       tracker.MeasureAssociate(measure_datas, robot_id);
    }
    tracker.RearrangeTracks();
    tracker.TeammateIdentify(teammates);
    tracker.GetObjects(fuse_obs_);

    self_tracker.MeasureAssociate(obs_measure_[AgentID_-1]);
    self_tracker.RearrangeTracks();
    self_tracker.GetAllObjects(self_obs_);
    int obstalce_nums = self_obs_.size();
    if (obstalce_nums == 0)
        return;
    sort_info_.resize(self_obs_.size());
    for(int i = 0 ; i < obstalce_nums;i++)
    {
        sort_info_[i].distance_  = self_obs_[i].distance(robot_pos_[AgentID_-1]);
        sort_info_[i].obstacle_  = self_obs_[i];
    }
    std::sort(sort_info_.begin(),sort_info_.end(),[](const obstacles_sort & obs1, const obstacles_sort & obs2)
      { return (obs1.distance_ < obs2.distance_);});
    for(int i = 0 ; i < obstalce_nums; i++)
        self_obs_[i]  = sort_info_[i].obstacle_;
}
