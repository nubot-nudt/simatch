#include "auto_referee.h"
#include <iostream>
using namespace std;

// Model info in world reference frame;
// Unit -- length: cm, velocity: cm/s, ori: rad, w: rad/s

auto_referee::auto_referee(int start_id)
{
    which_team_ = NONE_TEAM;
    start_team_ = start_id;
    cyan_score_ = 0;
    magenta_score_ = 0;
    currentCmd_ = STOPROBOT;
    nextCmd_ = STOPROBOT;
    dribble_id_ = -1;
    last_dribble_id_ = -1;
    ball_initpos_ = DPoint(0.0, 0.0);
    ball_resetpos_  = DPoint(0.0, 0.0);
    ModelStatesCB_flag_ = false;
    kickoff_flg_ = false;

    cyan_info_.reserve(OUR_TEAM);
    magenta_info_.reserve(OUR_TEAM);

    rosnode_ = new ros::NodeHandle();
    rosnode_->param("/cyan/prefix",     cyan_prefix_,      std::string("nubot"));
    rosnode_->param("/magenta/prefix",  magenta_prefix_,   std::string("rival"));
    rosnode_->param("/football/name",   ball_name_,          std::string("football") );

    /** ROS publishers **/
    cyan_pub_ = rosnode_->advertise<nubot_common::CoachInfo>("/"+cyan_prefix_+"/receive_from_coach", 100);
    magenta_pub_ = rosnode_->advertise<nubot_common::CoachInfo>("/"+magenta_prefix_+"/receive_from_coach", 100);

    /** ROS subscribers using custom callback queues and a thread **/
    ros::SubscribeOptions so1 = ros::SubscribeOptions::create<gazebo_msgs::ContactsState>(
                "/football/bumper_states", 100, boost::bind(&auto_referee::contactCallback ,this,_1),
                ros::VoidPtr(), &message_queue_);
    bumper_sub_ = rosnode_->subscribe(so1);
    ros::SubscribeOptions so2 = ros::SubscribeOptions::create<gazebo_msgs::ModelStates>(
                "/gazebo/model_states", 100, boost::bind(&auto_referee::msCallback ,this,_1),
                ros::VoidPtr(), &message_queue_);
    gazebo_sub_ = rosnode_->subscribe(so2);
//     bumper_sub_ = rosnode_->subscribe("/football/bumper_states", 10, &auto_referee::contactCallback, this);
//    gazebo_sub_ = rosnode_->subscribe("/gazebo/model_states", 10, &auto_referee::msCallback, this);

    /** ROS sercice client **/
    ms_client_ = rosnode_->serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    /** ROS service server using custom callback queues and a thread **/
    /** IMPORTATN: use a custom queue and a thread to process service calls to avoid deadlock **/
    ros::AdvertiseServiceOptions aso1 = ros::AdvertiseServiceOptions::create<nubot_common::DribbleId>(
                "DribbleId", boost::bind(&auto_referee::dribbleService, this, _1, _2),
                ros::VoidPtr(), &service_queue_);
    dribble_server_ =   rosnode_->advertiseService(aso1);

    /** Custom Callback Queue Thread. Use threads to process message and service callback queue **/
    service_callback_queue_thread_ = boost::thread(boost::bind(&auto_referee::service_queue_thread, this));
    message_callback_queue_thread_ = boost::thread( boost::bind( &auto_referee::message_queue_thread,this ) );

    /** timer process **/
    loop_timer_ = rosnode_->createTimer(ros::Duration(LOOP_PERIOD), &auto_referee::loopControl, this);
    createRecord();
}

auto_referee::~auto_referee()
{
    message_queue_.clear();
    service_queue_.clear();
    message_queue_.disable();
    service_queue_.disable();
    rosnode_->shutdown();
    message_callback_queue_thread_.join();
    service_callback_queue_thread_.join();
    record_.close();
    delete rosnode_;
}

void auto_referee::loopControl(const ros::TimerEvent &event)
{
    msgCB_lock_.lock();
    srvCB_lock_.lock();

    if(ros::ok() && ModelStatesCB_flag_)        // available to get the model states
    {
        isGameStart();
        if(currentCmd_!=STOPROBOT)
        {
            isDribbleFault();
            R3_detectBallOut();
            R5_isOppGoal_PenaltyArea();
        }
        else
        {
#if 0
            if(waittime(2))
                sendGameCommand(nextCmd_);
            else if(dribble_id_ == -1)      // dribble request is received
                setBallPos(ball_resetpos_.x_, ball_resetpos_.y_);
#else
            if(waittime(2))
            {
                setBallPos(ball_resetpos_.x_, ball_resetpos_.y_);
                sendGameCommand(nextCmd_);
            }
#endif
        }

        //sendGameCommand(currentCmd_);       // send every time to make sure game cmds are received
    }

    srvCB_lock_.unlock();
    msgCB_lock_.unlock();
}

void auto_referee::msCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
    msgCB_lock_.lock();

    ModelStatesCB_flag_ = true;
    int num = msg->name.size();
    cyan_info_.clear();
    magenta_info_.clear();

    // Model info in world reference frame;
    // Unit -- length: cm, velocity: cm/s, ori: rad, w: rad/s
    for(int i=0; i<num;i++)
    {
        std::string name = msg->name[i];
        gazebo::math::Quaternion    qua(msg->pose[i].orientation.w, msg->pose[i].orientation.x,
                                        msg->pose[i].orientation.y, msg->pose[i].orientation.z);
        ModelState  ms;
        ms.name = msg->name[i];
        ms.pos.x_ = msg->pose[i].position.x * M2CM_CONVERSION;
        ms.pos.y_ = msg->pose[i].position.y * M2CM_CONVERSION;
        ms.pos_z  = msg->pose[i].position.z * M2CM_CONVERSION;
        ms.ori    = qua.GetYaw();
        ms.vel.x_ = msg->twist[i].linear.x * M2CM_CONVERSION;
        ms.vel.y_ = msg->twist[i].linear.y * M2CM_CONVERSION;
        ms.w      = msg->twist[i].angular.z;

        if(name.find(cyan_prefix_) != std::string::npos)
        {
            ms.id = atoi( ms.name.substr(cyan_prefix_.size()).c_str() );
            cyan_info_.push_back(ms);
        }
        else if(name.find(magenta_prefix_) != std::string::npos)
        {
            ms.id = atoi( ms.name.substr(magenta_prefix_.size()).c_str() );
            ms.ori = ms.ori > 0.0 ? (ms.ori - M_PI) :  (ms.ori + M_PI);     // since rival models' body frame is flipped
            magenta_info_.push_back(ms);
        }
        else if(name.find(ball_name_) != std::string::npos)
            ball_state_ = ms;
    }

    msgCB_lock_.unlock();
}

void auto_referee::contactCallback(const gazebo_msgs::ContactsState::ConstPtr &msg)
{
    msgCB_lock_.lock();

    contacts_ = *msg;
    for(int i =0; i<contacts_.states.size();i++)
    {
        std::string coll = contacts_.states[i].collision2_name;
        std::size_t cyan_found = coll.find(cyan_prefix_);
        std::size_t magenta_found = coll.find(magenta_prefix_);

        if(cyan_found != std::string::npos)
            which_team_ = CYAN_TEAM;
        else if(magenta_found != std::string::npos)
            which_team_ = MAGENTA_TEAM;
    }

    msgCB_lock_.unlock();
}

bool auto_referee::dribbleService(nubot_common::DribbleId::Request &req,
                                  nubot_common::DribbleId::Response &res)
{
    srvCB_lock_.lock();

    dribble_id_ = req.AgentId;
    if(dribble_id_ <= 5)
        which_team_ = CYAN_TEAM;
    else
        which_team_ = MAGENTA_TEAM;

    srvCB_lock_.unlock();
    return true;
}

bool auto_referee::isGameStart()
{
    static bool once = true;

    if(once)       // only run at the beginning of the match
    {
        if(start_team_ == CYAN_TEAM || start_team_ == MAGENTA_TEAM)
        {
            sendGameCommand(STOPROBOT);
            nextCmd_ = (start_team_==CYAN_TEAM) ? OUR_KICKOFF : OPP_KICKOFF;
            kickoff_flg_ = true;
            once = false;
        }
    }
    else if( (currentCmd_!=PARKINGROBOT) && (currentCmd_!=STOPROBOT) && (currentCmd_!=STARTROBOT))    // start game
    {
        if(waittime(3))
            sendGameCommand(STARTROBOT);
        return true;
    }

    return false;
}

int auto_referee::isDribbleFault()
{
    int rtnv = 0;  // return value

    if(dribble_id_ != -1)    // it means the ball is dribbled by a robot
    {
        if(dribble_id_ <= 5)
            getModelState(CYAN_TEAM, dribble_id_, track_ms_);
        else
            getModelState(MAGENTA_TEAM, dribble_id_-5, track_ms_);

        if(last_dribble_id_ != dribble_id_)
        {
            ball_initpos_ = ball_state_.pos;
            ROS_INFO("ball init pos:[%.1f, %.1f]", ball_initpos_.x_, ball_initpos_.y_);
        }

        if(R1_isDribble3m())
            rtnv = 1;
        if(R2_isDribbleCrossField())
            rtnv = 2;
    }

    last_dribble_id_ = dribble_id_;
    return rtnv;
}

bool auto_referee::R1_isDribble3m()
{
    if(ball_initpos_.distance(ball_state_.pos) > 300)
    {
        ball_resetpos_ = ball_state_.pos;
        sendGameCommand(STOPROBOT);
        nextCmd_ = (which_team_==CYAN_TEAM)? OPP_FREEKICK : OUR_FREEKICK;
        writeRecord(track_ms_.name+" dribbles more than 300 cm");
        ROS_INFO("ball init pos:[%.1f, %.1f], pos:[%.1f, %.1f]", ball_initpos_.x_, ball_initpos_.y_,
                                                                 ball_state_.pos.x_, ball_state_.pos.y_);
        return true;
    }
    else
        return false;
}

bool auto_referee::R2_isDribbleCrossField()
{
    // Don't check this rule when kickoff, after this, always check this rule;
    if(sgn(ball_initpos_.x_) != sgn(ball_state_.pos.x_) && sgn(ball_initpos_.x_) != 0)
    {
        if(!kickoff_flg_)
        {
            ball_resetpos_ = ball_state_.pos;
            ROS_INFO("rest pt:%.1f %.1f",ball_resetpos_.x_, ball_resetpos_.y_);
            sendGameCommand(STOPROBOT);
            nextCmd_ = (which_team_==CYAN_TEAM)? OPP_FREEKICK : OUR_FREEKICK;
            writeRecord(track_ms_.name+" dribbles across the field");
            return true;
        }
        else
        {
            kickoff_flg_ = false;
            return false;
        }
    }
    else
        return false;
}

bool auto_referee::R5_isOppGoal_PenaltyArea()
{
    int cyan_num=0, magen_num=0;

    for(ModelState ms : cyan_info_)
    {
        if(fieldinfo_.isOppGoal(ms.pos))
        {
            ball_resetpos_ = DPoint(0.0,0.0);
            sendGameCommand(STOPROBOT);
            nextCmd_ = OPP_FREEKICK;
            writeRecord(ms.name+" in opp goal area!");
            return true;
        }
        else if(fieldinfo_.isOppPenalty(ms.pos))
            cyan_num++;

        if(cyan_num >=2)
        {
            ball_resetpos_ = DPoint(0.0,0.0);
            sendGameCommand(STOPROBOT);
            nextCmd_ = OPP_FREEKICK;
            writeRecord("two cyan robots in opp penalty area!");
            return true;
        }
    }

    for(ModelState ms : magenta_info_)
    {
        if(fieldinfo_.isOurGoal(ms.pos))
        {
            ball_resetpos_ = DPoint(0.0,0.0);
            sendGameCommand(STOPROBOT);
            nextCmd_ = OUR_FREEKICK;
            writeRecord(ms.name+" in opp goal area!");
            return true;
        }
        else if(fieldinfo_.isOppPenalty(ms.pos))
            magen_num++;

        if(magen_num >=2)
        {
            ball_resetpos_ = DPoint(0.0,0.0);
            sendGameCommand(STOPROBOT);
            nextCmd_ = OUR_FREEKICK;
            writeRecord("two magenta robots in opp penalty area!");
            return true;
        }
    }
    return false;
}

bool auto_referee::R3_detectBallOut()
{
    if( fieldinfo_.isOutBorder(LEFTBORDER, ball_state_.pos) || fieldinfo_.isOutBorder(RIGHTBORDER, ball_state_.pos) ||
            fieldinfo_.isOutBorder(UPBORDER, ball_state_.pos) || fieldinfo_.isOutBorder(DOWNBORDER, ball_state_.pos) )
    {
        ROS_INFO("ball out pos:%f %f",ball_state_.pos.x_, ball_state_.pos.y_);
        if(!R4_detectGoal())
        {
            if(which_team_ == CYAN_TEAM)
            {
                sendGameCommand(STOPROBOT);
                nextCmd_ = OPP_THROWIN;
                ball_resetpos_ = DPoint(0.0, 0.0);
                writeRecord("Cyan collides ball out");
            }
            else if(which_team_ == MAGENTA_TEAM)
            {
                sendGameCommand(STOPROBOT);
                nextCmd_ = OUR_THROWIN;
                ball_resetpos_ = DPoint(0.0, 0.0);
                writeRecord("Magenta collides ball out");
            }
            else
            {
                sendGameCommand(STOPROBOT);
                nextCmd_ = DROPBALL;
                ball_resetpos_ = DPoint(0.0, 0.0);
                writeRecord("Cannot determine cyan or magenta collides ball out");
            }
            return true;
        }
    }

    return false;
}

bool auto_referee::R4_detectGoal()
{
    static std::string s;
    if(fieldinfo_.isOutBorder(LEFTBORDER, ball_state_.pos) && fabs(ball_state_.pos.y_) < 100.0-BALL_RADIUS
                                                           && fabs(ball_state_.pos_z) < 87.5-BALL_RADIUS)    // magenta goals
    {
        if(currentCmd_ != OUR_KICKOFF)
        {
            sendGameCommand(STOPROBOT);
            magenta_score_++;
            nextCmd_ = OUR_KICKOFF;
            ball_resetpos_ = DPoint(0.0, 0.0);
            s="Cyan : Magenta ["+ std::to_string(cyan_score_)+" : "+ std::to_string(magenta_score_) +"]\t Magenta goals. ";
            writeRecord(s);
        }
        return true;
    }
    else if(fieldinfo_.isOutBorder(RIGHTBORDER, ball_state_.pos)  && fabs(ball_state_.pos.y_) < 100.0-BALL_RADIUS
                                                                  && fabs(ball_state_.pos_z) < 87.5-BALL_RADIUS)  // cyan goals
    {
        if(currentCmd_ != OPP_KICKOFF)
        {
            sendGameCommand(STOPROBOT);
            cyan_score_++;
            nextCmd_ = OPP_KICKOFF;
            ball_resetpos_ = DPoint(0.0, 0.0);
            s="Cyan : Magenta ["+ std::to_string(cyan_score_)+" : "+ std::to_string(magenta_score_) +"]\t Cyan goals. ";
            writeRecord(s);
        }
        return true;
    }

    return false;
}

bool auto_referee::setBallPos(double x, double y)
{
    gazebo_msgs::SetModelState  ms;
    ms.request.model_state.model_name = ball_name_;
    ms.request.model_state.pose.position.x = x * CM2M_CONVERSION;
    ms.request.model_state.pose.position.y = y * CM2M_CONVERSION;
    ms.request.model_state.pose.position.z = 0.12;
    ms.request.model_state.twist.linear.x = 0.0;
    ms.request.model_state.twist.linear.y = 0.0;
    ms.request.model_state.twist.linear.z = 0.0;
    ms.request.model_state.twist.angular.x = 0.0;
    ms.request.model_state.twist.angular.y = 0.0;
    ms.request.model_state.twist.angular.z = 0.0;
    if(ms_client_.call(ms))
        return true;
    else
        return false;
}

bool auto_referee::getModelState(int which_team, int id, ModelState &ms)
{
    if(which_team == CYAN_TEAM)
    {
        for(ModelState mss : cyan_info_)
            if(id == mss.id)
                ms = mss;
        return true;
    }
    else if(which_team == MAGENTA_TEAM)
    {
        for(ModelState mss : magenta_info_)
            if(id == mss.id)
                ms = mss;
        return true;
    }
    else
        ROS_ERROR("Please specify an appropriate team");

    return false;
}

void auto_referee::sendGameCommand(int id)
{
    static int PreCyanMode = id, PreMagentaMode = id;

    cyan_coach_info_.MatchMode = id;
    switch (id)
    {
        case STOPROBOT:
            magenta_gameCmd_.MatchMode = id;
            writeRecord("(cmd) STOP");
            break;
        case STARTROBOT:
            magenta_gameCmd_.MatchMode = id;
            writeRecord("(cmd) START");
            break;
        case DROPBALL:
            magenta_gameCmd_.MatchMode = id;
            writeRecord("(cmd) DROPBALL");
            break;
        case PARKINGROBOT:
            magenta_gameCmd_.MatchMode = id;
            writeRecord("(cmd) PARKING");
            break;
        case OUR_KICKOFF:
            magenta_gameCmd_.MatchMode = OPP_KICKOFF;
            writeRecord("(cmd) CYAN KICKOFF");
            break;
        case OPP_KICKOFF:
            magenta_gameCmd_.MatchMode = OUR_KICKOFF;
            writeRecord("(cmd) MAGENTA KICKOFF");
            break;
        case OUR_THROWIN:
            magenta_gameCmd_.MatchMode = OPP_THROWIN;
            writeRecord("(cmd) CYAN THROWIN");
            break;
        case OPP_THROWIN:
            magenta_gameCmd_.MatchMode = OUR_THROWIN;
            writeRecord("(cmd) MAGENTA THROWIN");
            break;
        case OUR_PENALTY:
            writeRecord("(cmd) CYAN PENALTY");
            magenta_gameCmd_.MatchMode = OPP_PENALTY;
            break;
        case OPP_PENALTY:
            magenta_gameCmd_.MatchMode = OUR_PENALTY;
            writeRecord("(cmd) MAGENTA PENALTY");
            break;
        case OUR_GOALKICK:
            magenta_gameCmd_.MatchMode = OPP_GOALKICK;
            writeRecord("(cmd) CYAN GOALKICK");
            break;
        case OPP_GOALKICK:
            magenta_gameCmd_.MatchMode = OUR_GOALKICK;
            writeRecord("(cmd) MAGENTA GOALKICK");
            break;
        case OUR_CORNERKICK:
            magenta_gameCmd_.MatchMode = OPP_CORNERKICK;
            writeRecord("(cmd) CYAN CORNERKICK");
            break;
        case OPP_CORNERKICK:
            magenta_gameCmd_.MatchMode = OUR_CORNERKICK;
            writeRecord("(cmd) MAGENTA CORNERKICK");
            break;
        case OUR_FREEKICK:
            magenta_gameCmd_.MatchMode = OPP_FREEKICK;
            writeRecord("(cmd) CYAN FREEKICK");
            break;
        case OPP_FREEKICK:
            magenta_gameCmd_.MatchMode = OUR_FREEKICK;
            writeRecord("(cmd) MAGENTA FREEKICK");
            break;
        default:
            magenta_gameCmd_.MatchMode = STOPROBOT;
            writeRecord("(cmd) STOP");
            break;
    }

    cyan_coach_info_.MatchType = PreCyanMode;
    magenta_gameCmd_.MatchType = PreMagentaMode;
    PreCyanMode = cyan_coach_info_.MatchMode;
    PreMagentaMode = magenta_gameCmd_.MatchMode;
    currentCmd_ = id;

    cyan_pub_.publish(cyan_coach_info_);
    magenta_pub_.publish(magenta_gameCmd_);
}

bool auto_referee::createRecord()
{
    std::string filename, dirname("record/");
    filename = dirname + cyan_prefix_ + "-" + magenta_prefix_ + "(" + getSysTime() + ").txt";

    boost::filesystem::path dir(dirname);
    if(!(boost::filesystem::exists(dir)))
        if (boost::filesystem::create_directory(dir))
            ROS_INFO("Successfully create directory: %s!", dir.c_str());

    record_.open(filename);
    if(record_.is_open())
    {
        ROS_INFO("Successfully create file: %s!", filename.c_str());
        return true;
    }
    else
    {
        ROS_ERROR("Failed create file: %s!", filename.c_str());
        return false;
    }
}

std::string auto_referee::getSysTime(std::string format)
{
    static int size = 50;
    time_t now=time(NULL);      // get time now
    char buffer[size];
    buffer[0] = '\0';

    if (now != -1)
       strftime(buffer, size, format.c_str(), localtime(&now));
    return std::string(buffer);
}

void auto_referee::writeRecord(string s)
{
    std::string ss = "[" + getSysTime("%T") + "] " +s + "\n";
    record_<<ss;
    //ROS_INFO("record: %s\n", s.c_str());
    cout<<s.c_str()<<endl;
}

int auto_referee::sgn(double x)
{
    if(gazebo::math::equal(x, 0.0, 10.0))   // error: +-10 cm
        return 0;
    else if(x < 0.0)
        return -1;
    else
        return 1;
}

bool auto_referee::waittime(double sec)
{
    static int count=0;
    if(count < sec/LOOP_PERIOD)     // wait 5 secs
    {
        count++;
        return false;
    }
    else
    {
        count = 0;
        return true;
    }
}

void auto_referee::service_queue_thread()
{
    static const double timeout = 0.01;
    while(rosnode_->ok())
        service_queue_.callAvailable(ros::WallDuration(timeout));
}

void auto_referee::message_queue_thread()
{
    static const double timeout = 0.01;
    while (rosnode_->ok())
        message_queue_.callAvailable(ros::WallDuration(timeout));
}

void auto_referee::test()
{
#if 1
    for(ModelState ms : cyan_info_)
        ROS_INFO("cyan_info:\n\tname:%s,\t id:%d\n \tpos:[%.0f, %.0f](cm), ori:%.0f(deg)\n \tvel:[%.0f,%.0f](cm/s), w:%.0f(deg/s)\n",
               ms.name.c_str(), ms.id, ms.pos.x_, ms.pos.y_, ms.ori*RAD2DEG, ms.vel.x_, ms.vel.y_, ms.w*RAD2DEG);
    for(ModelState ms : magenta_info_)
        ROS_INFO("magenta_info:\n\tname:%s,\t id:%d\n \tpos:[%.0f, %.0f](cm), ori:%.0f(deg)\n \tvel:[%.0f,%.0f](cm/s), w:%.0f(deg/s)\n",
               ms.name.c_str(), ms.id, ms.pos.x_, ms.pos.y_, ms.ori*RAD2DEG, ms.vel.x_, ms.vel.y_, ms.w*RAD2DEG);
#endif
#if 0
        //detectBallOut();
        detectGoal();
#endif
}

int main(int argc, char **argv)
{
    int id = 0;
    if(argc >=2)
        id = atoi(argv[1]);
    else if(argc < 2 || (id!=CYAN_TEAM && id!=MAGENTA_TEAM))
    {
        cout<<"Please specify who kicks off. "<<CYAN_TEAM<<" for cyan; "<<MAGENTA_TEAM<<" for magenta"<<endl;
        return -1;
    }

    ros::init(argc,argv,"auto_referee");
    ros::Time::init();
    ROS_INFO("start auto referee");

    auto_referee ref(id);
    ros::spin();
    return 0;
}

