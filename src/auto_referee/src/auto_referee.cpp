#include "auto_referee.h"
#include <iostream>

#ifdef USE_NCURSES
    #include <ncurses.h>
    #define OUTINFO     printw
    #define OUTERROR    printw
#else
    #define OUTINFO     ROS_INFO
    #define OUTERROR    ROS_ERROR
    #define KEY_UP      '5'
    #define KEY_DOWN    '2'
    #define KEY_LEFT    '1'
    #define KEY_RIGHT   '3'
#endif

#define WAIT_SECS_STOP      2   // after 'STOP' command is received, wait how many seconds before next command is sent
#define WAIT_SECS_PREGAME   4   // after pre-game command, such as free-kick, is received, wait how many seconds before next command is sent
using namespace std;
//zdx_note;for sendingoff and sendingback justice
int n2_isvalid_flag =1;
int n3_isvalid_flag =1;
int n4_isvalid_flag =1;
int n5_isvalid_flag =1;
int r2_isvalid_flag =1;
int r3_isvalid_flag =1;
int r4_isvalid_flag =1;
int r5_isvalid_flag =1;


// Model info in world reference frame;
// Unit -- length: cm, velocity: cm/s, ori: rad, w: rad/s

auto_referee::auto_referee(int start_id)
{
    #ifdef USE_NCURSES
    // ncurses initialization
    initscr();
    cbreak();               // one-character-a-time
    keypad(stdscr, TRUE);   // capture special keys
    nodelay(stdscr, TRUE);  // non-blocking input
    noecho();               // no echo
    scrollok(stdscr, TRUE); // automatic scroll screen
    OUTINFO("start auto referee\n");
    #endif

    lastTouchBallTeam_ = NONE_TEAM;
    start_team_ = start_id;
    cyan_score_ = 0;
    magenta_score_ = 0;
    currentCmd_ = STOPROBOT;
    nextCmd_ = STOPROBOT;
    dribble_id_ = -1;
    last_dribble_id_ = -1;
    robot_initpos_ = DPoint(0.0, 0.0);
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
    setMS_pub_ = rosnode_->advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 100);

    //zdx_note
    cyan_sending_off_pub = rosnode_->advertise<nubot_common::SendingOff>("/"+cyan_prefix_+"/redcard/chatter", 100);
    magenta_sending_off_pub = rosnode_->advertise<nubot_common::SendingOff>("/"+magenta_prefix_+"/redcard/chatter", 100);

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
    // setMS_client_ = rosnode_->serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    /** ROS service server using custom callback queues and a thread **/
    /** IMPORTATN: use a custom queue and a thread to process service calls to avoid deadlock **/
    ros::AdvertiseServiceOptions aso1 = ros::AdvertiseServiceOptions::create<nubot_common::DribbleId>(
                "DribbleId", boost::bind(&auto_referee::dribbleService, this, _1, _2),
                ros::VoidPtr(), &service_queue_);
    dribble_server_ =   rosnode_->advertiseService(aso1);

    /** Custom Callback Queue Thread. Use threads to process message and service callback queue **/
    service_callback_queue_thread_ = boost::thread(boost::bind(&auto_referee::service_queue_thread, this));
    message_callback_queue_thread_ = boost::thread(boost::bind( &auto_referee::message_queue_thread,this ));

    /** timer process **/
    loop_timer_ = rosnode_->createTimer(ros::Duration(LOOP_PERIOD), &auto_referee::loopControl, this);

    createRecord();
    // send the first game start command
    if(start_team_ == CYAN_TEAM || start_team_ == MAGENTA_TEAM)
    {
        sendGameCommand(STOPROBOT);
        nextCmd_ = (start_team_==CYAN_TEAM) ? OUR_KICKOFF : OPP_KICKOFF;
        kickoff_flg_ = true;
    }
}

auto_referee::~auto_referee()
{
    #ifdef USE_NCURSES
    endwin();                   // restore the terminal settings.
    #endif

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
#if 1
    ModelStatesCB_flag_ = true;
    static int err_count = 0;

    if(!isManualControl())                        // not manual control
    {
        if(ros::ok())
        {
            if(ModelStatesCB_flag_)                 // available to get the model states
            {
                if(currentCmd_ == STOPROBOT)
                {
                    if(waittime(WAIT_SECS_STOP))
                    {
                        setBallPos(ball_resetpos_.x_, ball_resetpos_.y_);
                        sendGameCommand(nextCmd_);
                    }
                }
                else if(currentCmd_ == STARTROBOT)
                {
                    // detect in-play faults
                    R1and2_isDribbleFault();
                    R3_isBallOutOrGoal();
                    R4_isOppGoalOrPenaltyArea();
                }
                else if( currentCmd_!=PARKINGROBOT) // pre-game commands, such as free-kick, drop-ball, etc.
                {
                    // detect set-play faults
                    if(waittime(WAIT_SECS_PREGAME))
                    {
                        //if(!R5_isTooCloseToBall())
                        sendGameCommand(STARTROBOT);
                    }
                    else
                    {
                        if(ball_state_.pos.distance(ball_resetpos_) > 10.0)     // prevent the ball from being kicked away
                            setBallPos(ball_resetpos_.x_, ball_resetpos_.y_);
                    }
                }
            }
            else
            {
                err_count++;
                if(err_count> 5.0/LOOP_PERIOD)          // model state flag is not valid for 5 secs
                {
                    OUTINFO("Err: model states NOT received. Has Gazebo started yet?\n");
                    err_count = 0;
                }
            }
        }
        else
            OUTINFO("Err: ros::ok() is false.\n");
    }
#else
    test();
#endif

    srvCB_lock_.unlock();
    msgCB_lock_.unlock();
}

bool auto_referee::isManualControl()
{
#ifdef USE_NCURSES
    static bool isManual = false;
    static int  pos_dif = 10;
    int ch;

    if( (ch=getch()) != ERR)       // user input
    {
        if(ch == ' ')
        {
            isManual = !isManual;
            if(isManual)
            {
                sendGameCommand(STOPROBOT);
                writeRecord("\nEnter the MANUAL mode. Press h or H for help!\n");
            }
            else
            {
                sendGameCommand(STARTROBOT);
                writeRecord("\nQuit the MANUAL mode!\n");
            }
        }

        if(isManual)
        {
            switch(ch)
            {
            case 'h':
            case 'H':
                printManualHelp();
                break;
            case 'k':
                sendGameCommand(OUR_KICKOFF);
                break;
            case 'K':
                sendGameCommand(OPP_KICKOFF);
                break;
            case 't':
                sendGameCommand(OUR_THROWIN);
                break;
            case 'T':
                sendGameCommand(OPP_THROWIN);
                break;
            case 'g':
                sendGameCommand(OUR_GOALKICK);
                break;
            case 'G':
                sendGameCommand(OPP_GOALKICK);
                break;
            case 'c':
                sendGameCommand(OUR_CORNERKICK);
                break;
            case 'C':
                sendGameCommand(OPP_CORNERKICK);
                break;
            case 'f':
                sendGameCommand(OUR_FREEKICK);
                break;
            case 'F':
                sendGameCommand(OPP_FREEKICK);
                break;
            case 'p':
                sendGameCommand(OUR_PENALTY);
                break;
            case 'P':
                sendGameCommand(OPP_PENALTY);
                break;
            case 'd':
                sendGameCommand(DROPBALL);
                break;
            case KEY_UP:
                setBallPos(ball_state_.pos.x_, ball_state_.pos.y_ + pos_dif);
                break;
            case KEY_DOWN:
                setBallPos(ball_state_.pos.x_, ball_state_.pos.y_ - pos_dif);
                break;
            case KEY_LEFT:
                setBallPos(ball_state_.pos.x_ - pos_dif, ball_state_.pos.y_);
                break;
            case KEY_RIGHT:
                setBallPos(ball_state_.pos.x_ + pos_dif, ball_state_.pos.y_);
                break;

                //zdx_note
            case '2':
                pub_sendingoff_flag.TeamName = "NuBot2";
                pub_sendingoff_flag.TeamInfo = 0;
                pub_sendingoff_flag.PlayerNum = 2;
                pub_sendingoff_flag.id_maxvel_isvalid = 2;
                n2_isvalid_flag ++;
                if(n2_isvalid_flag >=3)
                {
                    pub_sendingoff_flag.id_maxvel_isvalid = 12;
                    n2_isvalid_flag = 1;
                }
                cyan_sending_off_pub.publish(pub_sendingoff_flag);

                break;
            case '3':
                pub_sendingoff_flag.TeamName = "NuBot3";
                pub_sendingoff_flag.TeamInfo = 0;
                pub_sendingoff_flag.PlayerNum = 3;
                pub_sendingoff_flag.id_maxvel_isvalid   =3;
                n3_isvalid_flag ++;
                if(n3_isvalid_flag >=3)
                {
                    pub_sendingoff_flag.id_maxvel_isvalid = 13;
                    n3_isvalid_flag = 1;
                }
                cyan_sending_off_pub.publish(pub_sendingoff_flag);
                break;
            case '4':
                pub_sendingoff_flag.TeamName = "NuBot4";
                pub_sendingoff_flag.TeamInfo = 0;
                pub_sendingoff_flag.PlayerNum = 4;
                pub_sendingoff_flag.id_maxvel_isvalid   =4;
                n4_isvalid_flag ++;
                if(n4_isvalid_flag >=3)
                {
                    pub_sendingoff_flag.id_maxvel_isvalid = 14;
                    n4_isvalid_flag = 1;
                }
                cyan_sending_off_pub.publish(pub_sendingoff_flag);
                break;
            case '5':
                pub_sendingoff_flag.TeamName = "NuBot5";
                pub_sendingoff_flag.TeamInfo = 0;
                pub_sendingoff_flag.PlayerNum = 5;
                pub_sendingoff_flag.id_maxvel_isvalid   =5;
                n5_isvalid_flag ++;
                if(n5_isvalid_flag >=3)
                {
                    pub_sendingoff_flag.id_maxvel_isvalid = 15;
                    n5_isvalid_flag = 1;
                }
                cyan_sending_off_pub.publish(pub_sendingoff_flag);
                break;
            case '6':
                pub_sendingoff_flag.TeamName = "rival2";
                pub_sendingoff_flag.id_maxvel_isvalid   =2;
                pub_sendingoff_flag.TeamInfo = 1;
                pub_sendingoff_flag.PlayerNum = 2;
                r2_isvalid_flag ++;
                if(r2_isvalid_flag >=3)
                {
                    pub_sendingoff_flag.id_maxvel_isvalid = 12;
                    r2_isvalid_flag = 1;
                }
                cyan_sending_off_pub.publish(pub_sendingoff_flag);
                magenta_sending_off_pub.publish(pub_sendingoff_flag);
                break;
            case '7':
                pub_sendingoff_flag.TeamName = "rival3";
                pub_sendingoff_flag.id_maxvel_isvalid   =3;
                pub_sendingoff_flag.TeamInfo = 1;
                pub_sendingoff_flag.PlayerNum = 3;
                r3_isvalid_flag ++;
                if(r3_isvalid_flag >=3)
                {
                    pub_sendingoff_flag.id_maxvel_isvalid = 13;
                    r3_isvalid_flag = 1;
                }
                cyan_sending_off_pub.publish(pub_sendingoff_flag);
                magenta_sending_off_pub.publish(pub_sendingoff_flag);
                break;
            case '8':
                pub_sendingoff_flag.TeamName = "rival4";
                pub_sendingoff_flag.id_maxvel_isvalid   =4;
                pub_sendingoff_flag.TeamInfo = 1;
                pub_sendingoff_flag.PlayerNum = 4;
                r4_isvalid_flag ++;
                if(r4_isvalid_flag >=3)
                {
                    pub_sendingoff_flag.id_maxvel_isvalid = 14;
                    r4_isvalid_flag = 1;
                }
                cyan_sending_off_pub.publish(pub_sendingoff_flag);
                magenta_sending_off_pub.publish(pub_sendingoff_flag);
                break;
            case '9':
                pub_sendingoff_flag.TeamName = "rival5";
                pub_sendingoff_flag.id_maxvel_isvalid   =5;
                pub_sendingoff_flag.TeamInfo = 1;
                pub_sendingoff_flag.PlayerNum = 5;
                r5_isvalid_flag ++;
                if(r5_isvalid_flag >=3)
                {
                    pub_sendingoff_flag.id_maxvel_isvalid = 15;
                    r5_isvalid_flag = 1;
                }
                cyan_sending_off_pub.publish(pub_sendingoff_flag);
                magenta_sending_off_pub.publish(pub_sendingoff_flag);
                break;
                //
            default:
                break;
            }
        }
    }

    if(isManual)
        return true;
    else
        return false;
#else
    return false;
#endif
}

void auto_referee::printManualHelp()
{
    OUTINFO("You could also use the arrow key to control the movement of the ball or press the keys as follow to send game commands.\n\n");
    OUTINFO("              cyan        magenta       \n");
    OUTINFO("kick-off        k             K         \n");
    OUTINFO("throw-in        t             T         \n");
    OUTINFO("goal-kick       g             G         \n");
    OUTINFO("corner-kick     c             C         \n");
    OUTINFO("free-kick       f             F         \n");
    OUTINFO("penalty         p             P         \n");
    OUTINFO("drop-ball       d             d         \n");
    OUTINFO("sending0ff/back    2/3/4/5          6/7/8/9      \n");
    OUTINFO("stop/start    space          space      \n");

    OUTINFO("\n");
    OUTINFO("Press h or H to get this help message\n");
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

    static int tmpTeam = NONE_TEAM;
    contacts_ = *msg;
    for(int i =0; i<contacts_.states.size();i++)
    {
        std::string coll = contacts_.states[i].collision2_name;
        std::size_t cyan_found = coll.find(cyan_prefix_);
        std::size_t magenta_found = coll.find(magenta_prefix_);

        if(cyan_found != std::string::npos)
            tmpTeam = CYAN_TEAM;
        else if(magenta_found != std::string::npos)
            tmpTeam = MAGENTA_TEAM;
    }

    if(dribble_id_ == -1)       // it means no robot is dribbling the ball;
        lastTouchBallTeam_ = tmpTeam;
    else
    {
        if(dribble_id_ <= 5)
            lastTouchBallTeam_ = CYAN_TEAM;
        else
            lastTouchBallTeam_ = MAGENTA_TEAM;
    }

    msgCB_lock_.unlock();
}

bool auto_referee::dribbleService(nubot_common::DribbleId::Request &req,
                                  nubot_common::DribbleId::Response &res)
{
    srvCB_lock_.lock();

    dribble_id_ = req.AgentId;

    if(dribble_id_ != -1)           // it means a robot is dribbling the ball
    {
        if(dribble_id_ <= 5)
            lastTouchBallTeam_ = CYAN_TEAM;
        else
            lastTouchBallTeam_ = MAGENTA_TEAM;
    }

    srvCB_lock_.unlock();
    return true;
}

int auto_referee::R1and2_isDribbleFault()
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
            robot_initpos_ = track_ms_.pos;//ball_state_.pos;
        }

        if(R1_isDribble3m())
            rtnv = 1;
//        if(R2_isDribbleCrossField())
//            rtnv = 2;
    }

    last_dribble_id_ = dribble_id_;
    return rtnv;
}

bool auto_referee::R1_isDribble3m()
{
    if(robot_initpos_.distance(track_ms_.pos) > 300)
    {
        ball_resetpos_ = getBallRstPtNotInPenalty(ball_state_.pos);
        sendGameCommand(STOPROBOT);
        nextCmd_ = (lastTouchBallTeam_==CYAN_TEAM)? OPP_FREEKICK : OUR_FREEKICK;
        writeRecord(track_ms_.name+" dribbles more than 300 cm");
        OUTINFO("robot init pos:[%.1f, %.1f], pos:[%.1f, %.1f]\n", robot_initpos_.x_, robot_initpos_.y_,
                                                                 track_ms_.pos.x_, track_ms_.pos.y_);
        return true;
    }
    else
        return false;
}

/*
bool auto_referee::R2_isDribbleCrossField()
{
    // Don't check this rule when kickoff, after this, always check this rule;
    if(sgn(ball_initpos_.x_) != sgn(ball_state_.pos.x_) && sgn(ball_initpos_.x_) != 0)
    {
        if(!kickoff_flg_)
        {
            ball_resetpos_ = ball_state_.pos;
            OUTINFO("rest pt:%.1f %.1f\n",ball_resetpos_.x_, ball_resetpos_.y_);
            sendGameCommand(STOPROBOT);
            nextCmd_ = (lastTouchBallTeam_==CYAN_TEAM)? OPP_FREEKICK : OUR_FREEKICK;
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
*/

bool auto_referee::R4_isOppGoalOrPenaltyArea()
{
    int cyan_num1=0, magen_num1=0, cyan_num2=0, magen_num2=0;

    for(ModelState ms : cyan_info_)
    {
        if(ms.id != 1)      // not consider the goal keeper
        {
            if(fieldinfo_.isOppGoal(ms.pos) || fieldinfo_.isOurGoal(ms.pos) ||
               isOurGoalPoleArea(ms.pos) || isOppGoalPoleArea(ms.pos))
            {
                ball_resetpos_ = getBallRstPtNotInPenalty(ball_state_.pos);
                sendGameCommand(STOPROBOT);
                nextCmd_ = OPP_FREEKICK;
                writeRecord(ms.name+" in the goal/goal_pole area!");
                return true;
            }

            if(fieldinfo_.isOurPenalty(ms.pos))
                cyan_num1++;

            if(fieldinfo_.isOppPenalty(ms.pos))
                cyan_num2++;

            if(cyan_num1 >=2 || cyan_num2 >= 2)   // if two or more robots are in penalty areas, then they violate the rule
            {
                ball_resetpos_ = getBallRstPtNotInPenalty(ball_state_.pos);
                sendGameCommand(STOPROBOT);
                nextCmd_ = OPP_FREEKICK;
                writeRecord("two or more cyan robots in the penalty area!");
                return true;
            }
        }
    }

    for(ModelState ms : magenta_info_)
    {
        if(ms.id != 1)      // not consider the goal keeper
        {
            if(fieldinfo_.isOppGoal(ms.pos) || fieldinfo_.isOurGoal(ms.pos) ||
               isOurGoalPoleArea(ms.pos) || isOppGoalPoleArea(ms.pos))
            {
                ball_resetpos_ = getBallRstPtNotInPenalty(ball_state_.pos);
                sendGameCommand(STOPROBOT);
                nextCmd_ = OUR_FREEKICK;
                writeRecord(ms.name+" in the goal/goal_pole area!");
                return true;
            }

            if(fieldinfo_.isOurPenalty(ms.pos))
                magen_num1++;

            if(fieldinfo_.isOppPenalty(ms.pos))
                magen_num2++;

            if(magen_num1 >=2 || magen_num2>=2)     // if two or more robots are in penalty areas, then they violate the rule
            {
                ball_resetpos_ = getBallRstPtNotInPenalty(ball_state_.pos);
                sendGameCommand(STOPROBOT);
                nextCmd_ = OUR_FREEKICK;
                writeRecord("two or more magenta robots in the penalty area!");
                return true;
            }
        }
    }
    return false;
}

bool auto_referee::R5_isTooCloseToBall()
{
    if(currentCmd_ == OUR_THROWIN || currentCmd_ == OUR_GOALKICK || currentCmd_ == OUR_CORNERKICK || currentCmd_ == OUR_FREEKICK)   // refered as 4-type-kick
    {
        int count=0;
        for(ModelState ms : magenta_info_)
        {
            if( !fieldinfo_.isOppPenalty(ms.pos) && ball_state_.pos.distance(ms.pos)<300.0)
            {
                sendGameCommand(STOPROBOT);
                writeRecord("magenta violates the 4-type-kick positioning rule");
                return true;
            }
        }
        for(ModelState ms : cyan_info_)
        {
            if(ball_state_.pos.distance(ms.pos)<200.0)
                count++;
            if(count>1)
            {
                sendGameCommand(STOPROBOT);
                writeRecord("cyan violates the 4-type-kick positioning rule");
                return true;
            }
        }
    }
    else if(currentCmd_ == OPP_THROWIN || currentCmd_ == OPP_GOALKICK || currentCmd_ == OPP_CORNERKICK || currentCmd_ == OPP_FREEKICK)
    {
        int count=0;
        for(ModelState ms : cyan_info_)
        {
            if( !fieldinfo_.isOurPenalty(ms.pos) && ball_state_.pos.distance(ms.pos)<300.0)
            {
                sendGameCommand(STOPROBOT);
                writeRecord("cyan violates the 4-type-kick positioning rule");
                return true;
            }
        }
        for(ModelState ms : magenta_info_)
        {
            if(ball_state_.pos.distance(ms.pos)<200.0)
                count++;
            if(count>1)
            {
                sendGameCommand(STOPROBOT);
                writeRecord("magenta violates the 4-type-kick positioning rule");
                return true;
            }
        }
    }
    else if(currentCmd_ == OUR_KICKOFF)
    {
        int count=0;
        for(ModelState ms : cyan_info_)
        {
            if( !(fieldinfo_.isOurField(ms.pos) && ball_state_.pos.distance(ms.pos)>200.0) )
                count++;
            if(count > 1)       // only the robot taking the kcik could violate the rule; but now more than 1 robot violate the rule;
            {
                sendGameCommand(STOPROBOT);
                writeRecord("cyan violates the kick-off positioning rule");
                return true;
            }
        }
        for(ModelState ms : magenta_info_)
        {
            if( !(fieldinfo_.isOppField(ms.pos) && ball_state_.pos.distance(ms.pos)>300.0) )
            {
                sendGameCommand(STOPROBOT);
                writeRecord("magenta violates the kick-off positioning rule");
                return true;
            }
        }
    }
    else if(currentCmd_ == OPP_KICKOFF)
    {
        int count=0;
        for(ModelState ms : magenta_info_)
        {
            if( !(fieldinfo_.isOurField(ms.pos) && ball_state_.pos.distance(ms.pos)>200.0) )
                count++;
            if(count > 1)       // only the robot taking the kcik could violate the rule; but now more than 1 robot violate the rule;
            {
                sendGameCommand(STOPROBOT);
                writeRecord("magenta violates the kick-off positioning rule");
                return true;
            }
        }
        for(ModelState ms : cyan_info_)
        {
            if( !(fieldinfo_.isOppField(ms.pos) && ball_state_.pos.distance(ms.pos)>300.0) )
            {
                sendGameCommand(STOPROBOT);
                writeRecord("cyan violates the kick-off positioning rule");
                return true;
            }
        }
    }
    else if(currentCmd_ == DROPBALL)
    {
        for(ModelState ms : magenta_info_)
        {
            if( !fieldinfo_.isOppPenalty(ms.pos) && ball_state_.pos.distance(ms.pos)<100.0)
            {
                sendGameCommand(STOPROBOT);
                writeRecord("magenta violates the drop-ball positioning rule");
                return true;
            }
        }
        for(ModelState ms : cyan_info_)
        {
            if( !fieldinfo_.isOurPenalty(ms.pos) && ball_state_.pos.distance(ms.pos)<100.0)
            {
                sendGameCommand(STOPROBOT);
                writeRecord("cyan violates the drop-ball positioning rule");
                return true;
            }
        }
    }
    else if(currentCmd_ == OUR_PENALTY )
    {
        int count=0;
        for(ModelState ms : magenta_info_)
        {
            if(ms.id != 1)
            {
                if( fieldinfo_.isOppPenalty(ms.pos) || fieldinfo_.isOurPenalty(ms.pos) ||  ball_state_.pos.distance(ms.pos)<300.0)
                {
                    sendGameCommand(STOPROBOT);
                    writeRecord("magenta violates the penalty-kick positioning rule");
                    return true;
                }
            }
        }
        for(ModelState ms : cyan_info_)
        {
            if(ms.id != 1)
            {
                if(ball_state_.pos.distance(ms.pos)<300.0)
                    count++;
                if(fieldinfo_.isOppPenalty(ms.pos) || fieldinfo_.isOurPenalty(ms.pos) || count>1)
                {
                    sendGameCommand(STOPROBOT);
                    writeRecord("cyan violates the penalty-kick positioning rule");
                    return true;
                }
            }
        }
    }
    else if(currentCmd_ == OPP_PENALTY)
    {
        int count=0;
        for(ModelState ms : cyan_info_)
        {
            if(ms.id != 1)
            {
                if( fieldinfo_.isOppPenalty(ms.pos) || fieldinfo_.isOurPenalty(ms.pos) ||  ball_state_.pos.distance(ms.pos)<300.0)
                {
                    sendGameCommand(STOPROBOT);
                    writeRecord("cyan violates the penalty-kick positioning rule");
                    return true;
                }
            }
        }
        for(ModelState ms : magenta_info_)
        {
            if(ms.id != 1)
            {
                if(ball_state_.pos.distance(ms.pos)<300.0)
                    count++;
                if(fieldinfo_.isOppPenalty(ms.pos) || fieldinfo_.isOurPenalty(ms.pos) || count>1)
                {
                    sendGameCommand(STOPROBOT);
                    writeRecord("magenta violates the penalty-kick positioning rule");
                    return true;
                }
            }
        }
    }

    R4_isOppGoalOrPenaltyArea();        // check if violates rule 4
    return false;
}

bool auto_referee::R3_isBallOutOrGoal()
{
    static std::string s;
    if( fieldinfo_.isOutBorder(LEFTBORDER, ball_state_.pos) )
    {
        OUTINFO("LEFT out pos:%f %f %f\n",ball_state_.pos.x_, ball_state_.pos.y_, ball_state_.pos_z);
        if(fabs(ball_state_.pos.y_) < fieldinfo_.ourGoal_[GOAL_UPPER].y_-BALL_RADIUS && fabs(ball_state_.pos_z) < GOAL_HEIGHT-BALL_RADIUS
                && ball_state_.pos.x_>-FIELD_LENGTH/2.0 - GOALPOST_WIDTH)    // magenta goals
        {
            if(currentCmd_ != OUR_KICKOFF)      // prevent the code from going into again
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
        else    // ball out
        {
            if(lastTouchBallTeam_ == CYAN_TEAM)
            {
                sendGameCommand(STOPROBOT);
                nextCmd_ = OPP_CORNERKICK;
                ball_resetpos_ = (ball_state_.pos.distance(LU_CORNER) < ball_state_.pos.distance(LD_CORNER))?
                            LU_CORNER : LD_CORNER;
                writeRecord("Cyan collides ball out");
            }
            else if(lastTouchBallTeam_ == MAGENTA_TEAM)
            {
                sendGameCommand(STOPROBOT);
                nextCmd_ = OUR_GOALKICK;
                ball_resetpos_ = (ball_state_.pos.distance(LU_RSTPT) < ball_state_.pos.distance(LD_RSTPT))?
                            LU_RSTPT : LD_RSTPT;
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
    else if(fieldinfo_.isOutBorder(RIGHTBORDER, ball_state_.pos))
    {
        OUTINFO("RIGHT out pos:%f %f %f\n",ball_state_.pos.x_, ball_state_.pos.y_, ball_state_.pos_z);
        if(fabs(ball_state_.pos.y_) < fieldinfo_.ourGoal_[GOAL_UPPER].y_-BALL_RADIUS && fabs(ball_state_.pos_z) < GOAL_HEIGHT-BALL_RADIUS
                && ball_state_.pos.x_<FIELD_LENGTH/2.0 + GOALPOST_WIDTH)  // cyan goals
        {
            if(currentCmd_ != OPP_KICKOFF)      // prevent the code from going into again
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
        else    // ball out
        {
            if(lastTouchBallTeam_ == CYAN_TEAM)
            {
                sendGameCommand(STOPROBOT);
                nextCmd_ = OPP_GOALKICK;
                ball_resetpos_ = (ball_state_.pos.distance(RU_RSTPT) < ball_state_.pos.distance(RD_RSTPT))?
                            RU_RSTPT : RD_RSTPT;
                writeRecord("Cyan collides ball out");
            }
            else if(lastTouchBallTeam_ == MAGENTA_TEAM)
            {
                sendGameCommand(STOPROBOT);
                nextCmd_ = OUR_CORNERKICK;
                ball_resetpos_ = (ball_state_.pos.distance(RU_CORNER) < ball_state_.pos.distance(RD_CORNER))?
                            RU_CORNER : RD_CORNER;
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
    else if(fieldinfo_.isOutBorder(UPBORDER, ball_state_.pos))
    {
        OUTINFO("UP out pos:%f %f %f\n",ball_state_.pos.x_, ball_state_.pos.y_, ball_state_.pos_z);
        ball_resetpos_ = DPoint(ball_state_.pos.x_, fieldinfo_.yline_[0]-30.0);
        if(lastTouchBallTeam_ == CYAN_TEAM)
        {
            sendGameCommand(STOPROBOT);
            nextCmd_ = OPP_THROWIN;
            writeRecord("Cyan collides ball out");
        }
        else if(lastTouchBallTeam_ == MAGENTA_TEAM)
        {
            sendGameCommand(STOPROBOT);
            nextCmd_ = OUR_THROWIN;
            writeRecord("Magenta collides ball out");
        }
        else
        {
            sendGameCommand(STOPROBOT);
            nextCmd_ = DROPBALL;
            writeRecord("Cannot determine cyan or magenta collides ball out");
        }
        return true;
    }
    else if(fieldinfo_.isOutBorder(DOWNBORDER, ball_state_.pos))
    {
        OUTINFO("DOWN out pos:%f %f %f\n",ball_state_.pos.x_, ball_state_.pos.y_, ball_state_.pos_z);
        ball_resetpos_ = DPoint(ball_state_.pos.x_, fieldinfo_.yline_[5]+30.0);
        if(lastTouchBallTeam_ == CYAN_TEAM)
        {
            sendGameCommand(STOPROBOT);
            nextCmd_ = OPP_THROWIN;
            writeRecord("Cyan collides ball out");
        }
        else if(lastTouchBallTeam_ == MAGENTA_TEAM)
        {
            sendGameCommand(STOPROBOT);
            nextCmd_ = OUR_THROWIN;
            writeRecord("Magenta collides ball out");
        }
        else
        {
            sendGameCommand(STOPROBOT);
            nextCmd_ = DROPBALL;
            writeRecord("Cannot determine cyan or magenta collides ball out");
        }
        return true;
    }

    return false;
}

bool auto_referee::isGoal()
{
    static std::string s;
    if(fieldinfo_.isOutBorder(LEFTBORDER, ball_state_.pos) && fabs(ball_state_.pos.y_) < fieldinfo_.ourGoal_[GOAL_UPPER].y_-BALL_RADIUS
            && fabs(ball_state_.pos_z) < GOAL_HEIGHT-BALL_RADIUS)    // magenta goals
    {
        if(currentCmd_ != OUR_KICKOFF)      // prevent the code from going into again
        {
            sendGameCommand(STOPROBOT);
            magenta_score_++;
            nextCmd_ = OUR_KICKOFF;
            ball_resetpos_ = DPoint(0.0, 0.0);
            s="Cyan : Magenta ["+ std::to_string(cyan_score_)+" : "+ std::to_string(magenta_score_) +"]\t Magenta goals. ";
            writeRecord(s);
        }
        std::cout<<fieldinfo_.ourGoal_[GOAL_UPPER].x_<<std::endl;
        return true;
    }
    else if(fieldinfo_.isOutBorder(RIGHTBORDER, ball_state_.pos)  && fabs(ball_state_.pos.y_) < fieldinfo_.ourGoal_[GOAL_UPPER].y_ - BALL_RADIUS
            && fabs(ball_state_.pos_z) < GOAL_HEIGHT-BALL_RADIUS)  // cyan goals
    {
        if(currentCmd_ != OPP_KICKOFF)      // prevent the code from going into again
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

bool auto_referee::isOurGoalPoleArea(DPoint world_pt)
{
    if(world_pt.x_ < -FIELD_LENGTH/2.0 && world_pt.x_ > -FIELD_LENGTH/2.0 - GOALPOST_WIDTH &&
       world_pt.y_ < GOALPOST_LEN/2.0 && world_pt.y_ > -GOALPOST_LEN/2.0)
        return true;
    else
        return false;
}

bool auto_referee::isOppGoalPoleArea(DPoint world_pt)
{
    if(world_pt.x_ > FIELD_LENGTH/2.0 && world_pt.x_ < FIELD_LENGTH/2.0 + GOALPOST_WIDTH &&
       world_pt.y_ < GOALPOST_LEN/2.0 && world_pt.y_ > -GOALPOST_LEN/2.0)
        return true;
    else
        return false;
}

bool auto_referee::setBallPos(double x, double y)
{
#if 0
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
    if(setMS_client_.call(ms))
        return true;
    else
        return false;
#else
    gazebo_msgs::ModelState ms;
    ms.model_name = ball_name_;
    ms.pose.position.x = x * CM2M_CONVERSION;
    ms.pose.position.y = y * CM2M_CONVERSION;
    ms.pose.position.z = 0.12;
    for(int i=0; i<2; i++)          // send serveral times to make sure the message is received
        setMS_pub_.publish(ms);
#endif
}

DPoint auto_referee::getBallRstPtNotInPenalty(DPoint ball_pos)
{
    if(!fieldinfo_.isOppPenalty(ball_pos) && !fieldinfo_.isOurPenalty(ball_pos))
        return ball_pos;
    else if(fieldinfo_.isOppPenalty(ball_pos))
        return (ball_pos.distance(RU_RSTPT) < ball_pos.distance(RD_RSTPT))?
                    RU_RSTPT : RD_RSTPT;
    else if(fieldinfo_.isOurPenalty(ball_pos))
        return (ball_pos.distance(LU_RSTPT) < ball_pos.distance(LD_RSTPT))?
                    LU_RSTPT : LD_RSTPT;
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
        OUTINFO("Please specify an appropriate team\n");

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
    case OUR_PENALTY:
        writeRecord("(cmd) CYAN PENALTY");
        magenta_gameCmd_.MatchMode = OPP_PENALTY;
        break;
    case OPP_PENALTY:
        magenta_gameCmd_.MatchMode = OUR_PENALTY;
        writeRecord("(cmd) MAGENTA PENALTY");
        break;
    case DROPBALL:
        magenta_gameCmd_.MatchMode = id;
        writeRecord("(cmd) DROPBALL");
        break;
    default:
        magenta_gameCmd_.MatchMode = STOPROBOT;
        writeRecord("(cmd) STOP");
        break;
    }

    cyan_coach_info_.MatchType = PreCyanMode;
    magenta_gameCmd_.MatchType = PreMagentaMode;

    //zdx_note  write solid here, should be modify, according to the coach's message.  2020.11.07
    //cyan_coach_info_.idA = 1;
    //cyan_coach_info_.idB = 169;
    cyan_coach_info_.angleA = 450;
    cyan_coach_info_.angleB = 10;
    //cyan_coach_info_.kickforce = 77;

    magenta_gameCmd_.angleA = 450;
    magenta_gameCmd_.angleB = 10;

    //zdx_note end.

    PreCyanMode = cyan_coach_info_.MatchMode;
    PreMagentaMode = magenta_gameCmd_.MatchMode;
    currentCmd_ = id;

    for(int i=0; i<5; i++)                      // send serveral times to make sure the message is received
    {
        cyan_pub_.publish(cyan_coach_info_);
        magenta_pub_.publish(magenta_gameCmd_);
    }
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

bool auto_referee::createRecord()
{
    std::string filename, dirname("record/");
    filename = dirname + cyan_prefix_ + "-" + magenta_prefix_ + "(" + getSysTime() + ").txt";

    boost::filesystem::path dir(dirname);
    if(!(boost::filesystem::exists(dir)))
        if (boost::filesystem::create_directory(dir))
            OUTINFO("Successfully create directory: %s!\n", dir.c_str());

    record_.open(filename);
    if(record_.is_open())
    {
        OUTINFO("Successfully create file: %s!\n", filename.c_str());
        return true;
    }
    else
    {
        OUTINFO("Failed create file: %s!\n", filename.c_str());
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
    OUTINFO("%s\n", s.c_str());
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
#if 0
    for(ModelState ms : cyan_info_)
        OUTINFO("cyan_info:\n\tname:%s,\t id:%d\n \tpos:[%.0f, %.0f](cm), ori:%.0f(deg)\n \tvel:[%.0f,%.0f](cm/s), w:%.0f(deg/s)\n",
               ms.name.c_str(), ms.id, ms.pos.x_, ms.pos.y_, ms.ori*RAD2DEG, ms.vel.x_, ms.vel.y_, ms.w*RAD2DEG);
    for(ModelState ms : magenta_info_)
        OUTINFO("magenta_info:\n\tname:%s,\t id:%d\n \tpos:[%.0f, %.0f](cm), ori:%.0f(deg)\n \tvel:[%.0f,%.0f](cm/s), w:%.0f(deg/s)\n",
               ms.name.c_str(), ms.id, ms.pos.x_, ms.pos.y_, ms.ori*RAD2DEG, ms.vel.x_, ms.vel.y_, ms.w*RAD2DEG);
#endif
#if 0
    //detectBallOut();
    detectGoal();
#endif
#if 0
    sendGameCommand(OUR_PENALTY);
    R5_isTooCloseToBall();
#endif
    OUTINFO("id:%d", dribble_id_);
    if(lastTouchBallTeam_ == CYAN_TEAM)
        OUTINFO("CYAN\n");
    else if(lastTouchBallTeam_ == MAGENTA_TEAM)
        OUTINFO("MAGENTA\n");
    else
        OUTINFO("NONE\n");
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

    auto_referee ref(id);
    ros::spin();
    return 0;
}
