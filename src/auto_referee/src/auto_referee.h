#ifndef AUTO_REFEREE_H
#define AUTO_REFEREE_H

#include <ros/ros.h>
#include <ros/callback_queue.h>         // Custom Callback Queue
#include <ros/subscribe_options.h>
#include "core.hpp"
#include "nubot_common/CoachInfo.h"
#include "nubot_common/Point2d.h"
#include "nubot_common/DribbleId.h"
#include "nubot_common/SendingOff.h"
#include "nubot/nubot_control/fieldinformation.h"
#include <gazebo_msgs/ContactState.h>
#include <gazebo_msgs/ContactsState.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include "Quaternion.hh"

#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <time.h>
#include <string>
#include <fstream>

// Unit -- length: cm, velocity: cm/s, ori: rad, w: rad/s
#define CM2M_CONVERSION 0.01
#define M2CM_CONVERSION 100

#define USE_NCURSES

using namespace nubot;
const int CYAN_TEAM = -1;
const int MAGENTA_TEAM = 1;
const int NONE_TEAM = 0;
const double RAD2DEG = 180.0/M_PI;
const double LOOP_PERIOD = 0.005;
const double GOAL_HEIGHT = 101.0;  // Though the real goal height is 100cm, the base height of the field is 1cm
const double BALL_RADIUS = 11.0;   // cm
const double GOALPOST_WIDTH = 75.0;     // the width of the goal post area (cm)
const double GOALPOST_LEN   = 240.0;    // the length of the goal post area (cm)

const double CORNER_X = FIELD_LENGTH/2 - 30.0;
const double CORNER_Y = FIELD_WIDTH/2 - 30.0;
const double RSTPT_X  = FIELD_LENGTH/2 - 360.0;
const double RSTPT_Y  = FIELD_WIDTH/4;
const DPoint RU_CORNER(CORNER_X, CORNER_Y);   // right up corner point
const DPoint RD_CORNER(CORNER_X, -CORNER_Y);
const DPoint LU_CORNER(-CORNER_X, CORNER_Y);
const DPoint LD_CORNER(-CORNER_X, -CORNER_Y);
const DPoint RU_RSTPT(RSTPT_X, RSTPT_Y);     // right up restart point
const DPoint RD_RSTPT(RSTPT_X, -RSTPT_Y);
const DPoint LU_RSTPT(-RSTPT_X, RSTPT_Y);
const DPoint LD_RSTPT(-RSTPT_X, -RSTPT_Y);

using namespace nubot_common;
using namespace nubot;

struct ModelState
{
    std::string name;
    int         id;
    DPoint      pos;
    double      pos_z;      // z-axis pos; only useful for ball info
    double      ori;
    DPoint      vel;
    double      w;

    ModelState()
    {
        name = "";
        id = -1;
        pos = DPoint(-1000.0,1000.0);
        pos_z   = 0.0;
        ori = 0.0;
        vel = DPoint(0.0,0.0);
        w   = 0.0;
    }
};

class auto_referee
{
public:
    auto_referee(int start_id);
    ~auto_referee();

    /// \brief send game commands to two teams of robots
    ///  \param]in] id -- game command ID based on CYAN team
    void sendGameCommand(int id);

    /// \brief contact sensor callback function
    void contactCallback(const gazebo_msgs::ContactsState::ConstPtr& msg);

    /// \brief gazebo model states callback function
    void msCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);

    bool dribbleService(nubot_common::DribbleId::Request& req, nubot_common::DribbleId::Response& res);

    int R1and2_isDribbleFault();

    /// \brief Does the robot dribble the ball for 3 meters?
    bool R1_isDribble3m();

    /// \brief Does robot dribble the ball across the field?
    // bool R2_isDribbleCrossField();

    /// \brief Does the robot pass the ball before shooting the goal?
    bool R2_isPassBeforeGoal();

    /// \brief Is the ball out of the field or a goal?
    /// \return CYAN_TEAM or MAGENTA_TEAM
    bool R3_isBallOutOrGoal();

    /// \brief is a goal?
    bool isGoal();

    /// \brief Is two robots in the penalty area or one robot in goal area?
    bool R4_isOppGoalOrPenaltyArea();

    /// \brief Are robots too close to the ball when game commands sent?
    bool R5_isTooCloseToBall();

    /// \brief set ball position
    /// \param[in] x,y -- x and y position
    /// \return 1 -- sucess; 0 -- fail
    bool setBallPos(double x, double y);

    /// \brief if ball_pos is not in any penalty areas, then return ball_pos;
    ///  else return the nearest restart point
    DPoint getBallRstPtNotInPenalty(DPoint ball_pos);

    void loopControl(const ros::TimerEvent& event);

    bool isManualControl();

    bool isOurGoalPoleArea(DPoint world_pt);        // check if world_pt is inside our(cyan) goal pole
    bool isOppGoalPoleArea(DPoint world_pt);        // check if world_pt is inside opp(magenta) goal pole

    void printManualHelp();

    bool getModelState(int which_team, int id, ModelState& ms);

    bool waittime(double sec);

    bool createRecord();

    std::string getSysTime(std::string format=string("%F %R"));

    void writeRecord(std::string s);

    int sgn(double x);

    /// \brief Custom service callback queue thread
    void service_queue_thread();

    /// \brief Custom message callback queue thread
    void message_queue_thread();

    /// \brief auto_referee test function
    void test();

private:
    ros::NodeHandle*            rosnode_;
    ros::Timer                  loop_timer_;

    ros::Publisher              cyan_sending_off_pub;
    ros::Publisher              magenta_sending_off_pub;
    ros::Publisher              cyan_pub_;
    ros::Publisher              magenta_pub_;
    ros::Publisher              setMS_pub_;            // set model state publisher
    ros::Subscriber             bumper_sub_;
    ros::Subscriber             gazebo_sub_;
    ros::ServiceClient          setMS_client_;          // set model state service client
    ros::ServiceServer          dribble_server_;
    std::string                 cyan_prefix_;
    std::string                 magenta_prefix_;
    std::string                 ball_name_;
    CoachInfo                   cyan_coach_info_;
    CoachInfo                   magenta_gameCmd_;
    gazebo_msgs::ContactsState  contacts_;
    std::vector<ModelState>     cyan_info_;
    std::vector<ModelState>     magenta_info_;
    ModelState                  ball_state_;
    ModelState                  track_ms_;          // model state that is being tracked if it violates rules
    FieldInformation            fieldinfo_;
    DPoint                      robot_initpos_;      // the initial pos of dribble
    DPoint                      ball_resetpos_;     // ball reset point
    int                         lastTouchBallTeam_;        // which team last contacts with the ball
    int                         cyan_score_;
    int                         magenta_score_;
    int                         currentCmd_;          // next game command
    int                         nextCmd_;
    int                         dribble_id_;
    int                         last_dribble_id_;
    int                         start_team_;                 // team id when game starts at the very beginning
    bool                        ModelStatesCB_flag_;         // Indicate receiving messages
    bool                        kickoff_flg_;                // when kickoff cmd sends, this is true;
    std::ofstream               record_;

    nubot_common::SendingOff    pub_sendingoff_flag;         //for sending off robot player

    boost::thread               message_callback_queue_thread_;     // Thead object for the running callback Thread.
    boost::thread               service_callback_queue_thread_;
    boost::mutex                msgCB_lock_;        // A mutex to lock access to fields that are used in ROS message callbacks
    boost::mutex                srvCB_lock_;        // A mutex to lock access to fields that are used in ROS service callbacks
    ros::CallbackQueue          message_queue_;     // Custom Callback Queue. Details see http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning
    ros::CallbackQueue          service_queue_;     // Custom Callback Queue

};

#endif // AUTO_REFEREE_H
