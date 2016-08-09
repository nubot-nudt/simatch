#include "auto_referee.h"

auto_referee::auto_referee()
{
    cyan_info_.name.reserve(5);
    cyan_info_.pose.reserve(5);
    cyan_info_.twist.reserve(5);
    magenta_info_.name.reserve(5);
    magenta_info_.pose.reserve(5);
    magenta_info_.twist.reserve(5);

    rosnode_ = new ros::NodeHandle();
    rosnode_->param("/cyan/prefix",     cyan_prefix_,      std::string("nubot"));
    rosnode_->param("/magenta/prefix",  magenta_prefix_,   std::string("rival"));
    rosnode_->param("/football/name",   ball_name_,          std::string("football") );

    cyan_pub_ = rosnode_->advertise<nubot_common::CoachInfo>("/"+cyan_prefix_+"/receive_from_coach", 100);
    magenta_pub_ = rosnode_->advertise<nubot_common::CoachInfo>("/"+magenta_prefix_+"/receive_from_coach", 100);
    bumper_sub_ = rosnode_->subscribe<gazebo_msgs::ContactsState>("/football/bumper_states", 2, &auto_referee::contactCallback, this);
    gazebo_sub_ = rosnode_->subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1, &auto_referee::modelCallback, this);
}

auto_referee::~auto_referee()
{
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

void auto_referee::modelCallback(const gazebo_msgs::ModelStates::ConstPtr &states)
{
    int num = states->name.size();

    for(int i=0; i<num;i++)
    {
        std::string name = states->name[i];
        if(name.find(cyan_prefix_) != std::string::npos)
        {
            cyan_info_.name.push_back(states->name[i]);
            cyan_info_.pose.push_back(states->pose[i]);
            cyan_info_.twist.push_back(states->twist[i]);
        }
        else if(name.find(magenta_prefix_) != std::string::npos)
        {
            magenta_info_.name.push_back(states->name[i]);
            magenta_info_.pose.push_back(states->pose[i]);
            magenta_info_.twist.push_back(states->twist[i]);
        }
        else if(name.find(ball_name_) != std::string::npos)
        {
            ball_state_.model_name = states->name[i];
            ball_state_.pose = states->pose[i];
            ball_state_.twist = states->twist[i];
        }
    }
}

void auto_referee::contactCallback(const gazebo_msgs::ContactsState::ConstPtr &contacts)
{
    contacts_ = *contacts;
}

int auto_referee::whichCollidesBall()
{
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

