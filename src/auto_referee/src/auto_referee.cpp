#include "auto_referee.h"

auto_referee::auto_referee()
{
    rosnode_ = new ros::NodeHandle();
    rosnode_->param("/cyan/prefix",     cyan_prefix_,      std::string("nubot"));
    rosnode_->param("/magenta/prefix",  magenta_prefix_,   std::string("rival"));

    cyan_pub_ = rosnode_->advertise<nubot_common::CoachInfo>("/"+cyan_prefix_+"/receive_from_coach", 100);
    magenta_pub_ = rosnode_->advertise<nubot_common::CoachInfo>("/"+magenta_prefix_+"/receive_from_coach", 100);
    //bumper_sub_ = rosnode_->subscribe<gazebo_msgs::ContactsState>("/football/bumper_states", 2, &auto_referee::contactCallback, this);

    ros::SubscribeOptions so = ros::SubscribeOptions::create<gazebo_msgs::ContactsState>(
      "/football/bumper_states", 100, boost::bind( &auto_referee::contactCallback,this,_1),
      ros::VoidPtr(), &message_queue_);
    bumper_sub_  = rosnode_->subscribe(so);

    message_callback_queue_thread_ = boost::thread( boost::bind( &auto_referee::message_queue_thread_fun,this ) );

}

auto_referee::~auto_referee()
{
    message_queue_.clear();
    message_queue_.disable();
    message_callback_queue_thread_.join();
    rosnode_->shutdown();
    delete rosnode_;
}

void auto_referee::sendGameCommand(int id)
{
    static int PreCyanMode = id, PreMagentaMode = id;

    cyan_coach_info_.MatchMode = id;
    switch (id)
    {
        case STOPROBOT:
        case STARTROBOT:
        case DROPBALL:
        case PARKINGROBOT:
            magenta_coach_info_.MatchMode = id;
            break;
        case OUR_KICKOFF:
            magenta_coach_info_.MatchMode = OPP_KICKOFF;
            break;
        case OPP_KICKOFF:
            magenta_coach_info_.MatchMode = OUR_KICKOFF;
            break;
        case OUR_THROWIN:
            magenta_coach_info_.MatchMode = OPP_THROWIN;
            break;
        case OPP_THROWIN:
            magenta_coach_info_.MatchMode = OUR_THROWIN;
            break;
        case OUR_PENALTY:
            magenta_coach_info_.MatchMode = OPP_PENALTY;
            break;
        case OPP_PENALTY:
            magenta_coach_info_.MatchMode = OUR_PENALTY;
            break;
        case OUR_GOALKICK:
            magenta_coach_info_.MatchMode = OPP_GOALKICK;
            break;
        case OPP_GOALKICK:
            magenta_coach_info_.MatchMode = OUR_GOALKICK;
            break;
        case OUR_CORNERKICK:
            magenta_coach_info_.MatchMode = OPP_CORNERKICK;
            break;
        case OPP_CORNERKICK:
            magenta_coach_info_.MatchMode = OUR_CORNERKICK;
            break;
        case OUR_FREEKICK:
            magenta_coach_info_.MatchMode = OPP_FREEKICK;
            break;
        case OPP_FREEKICK:
            magenta_coach_info_.MatchMode = OUR_FREEKICK;
            break;
        default:
            magenta_coach_info_.MatchMode = STOPROBOT;
            break;
    }
    cyan_coach_info_.MatchType = PreCyanMode;
    magenta_coach_info_.MatchType = PreMagentaMode;
    PreCyanMode = cyan_coach_info_.MatchMode;
    PreMagentaMode = magenta_coach_info_.MatchMode;

    cyan_pub_.publish(cyan_coach_info_);
    magenta_pub_.publish(magenta_coach_info_);
}

void auto_referee::contactCallback(const gazebo_msgs::ContactsState::ConstPtr &contacts)
{
    msgCB_lock_.lock();

    contacts_ = *contacts;

    msgCB_lock_.unlock();
}

void auto_referee::message_queue_thread_fun()
{
    static const double timeout = 0.01;
    while (rosnode_->ok())
    {
      // Invoke all callbacks currently in the queue. If a callback was not ready to be called,
      // pushes it back onto the queue. This version includes a timeout which lets you specify
      // the amount of time to wait for a callback to be available before returning.
      message_queue_.callAvailable(ros::WallDuration(timeout));
    }
}

int auto_referee::whichCollidesBall()
{
    msgCB_lock_.lock();

    int which_team = 0;
    for(int i =0; i<contacts_.states.size();i++)
    {
        std::string coll = contacts_.states[i].collision2_name;
        std::size_t cyan_found = coll.find(cyan_prefix_);
        std::size_t magenta_found = coll.find(magenta_prefix_);

        if(cyan_found != std::string::npos)
            which_team = CYAN_TEAM;
        else if(magenta_found != std::string::npos)
            which_team = MAGENTA_TEAM;
    }

    if(which_team == CYAN_TEAM)
        ROS_INFO("cyan collides");
    else if(which_team == MAGENTA_TEAM)
        ROS_INFO("magenta collides");
    else
        ROS_INFO("not cyan or magenta");

    return which_team;

    msgCB_lock_.unlock();
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"auto_referee");
    ros::Time::init();
    ROS_INFO("start auto referee");
    auto_referee ref;
    ros::spin();
    return 0;
}

