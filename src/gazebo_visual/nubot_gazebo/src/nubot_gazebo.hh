#ifndef NUBOT_GAZEBO_HH
#define NUBOT_GAZEBO_HH

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>             // the core gazebo header files, including gazebo/math/gzmath.hh
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
// #include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <sdf/sdf.hh>

#include <ros/callback_queue.h>         // Custom Callback Queue
#include <ros/subscribe_options.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ModelState.h>
#include <ignition/math.hh>
#include "nubot_common/ActionCmd.h"
#include "nubot_common/OminiVisionInfo.h"
#include "nubot_common/VelCmd.h"
#include "nubot_common/BallIsHolding.h"
#include "nubot_common/DribbleId.h"
#include "nubot_common/CoachInfo.h"
#include "nubot_common/SendingOff.h"
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <string>

#include "core.hpp"

#include <nubot_gazebo/NubotGazeboConfig.h>
#include <dynamic_reconfigure/server.h>
using namespace ignition;

enum nubot_state
{
    CHASE_BALL,
    DRIBBLE_BALL,
    KICK_BALL,
    RESET
};

enum nubot_substate
{
    MOVE_BALL,
    ROTATE_BALL
};

namespace gazebo{
   struct Pose
   {
       math::Vector3d   position;
       math::Quaterniond orient;
   };
   struct Twist
   {
       math::Vector3d     linear;
       math::Vector3d     angular;
   };

   struct model_state
   {
       std::string model_name;
       Pose        pose;
       Twist       twist;
       std::string reference_frame;
   };                                       // similar to the type gazebo_msgs::ModelState;
                                            // use these structs for easy handling of model states.
   struct Obstacles
   {
       std::vector<nubot::PPoint> real_obs_;
       std::vector<nubot::DPoint> world_obs_;
   };

  class NubotGazebo : public ModelPlugin
  {
    private:

        physics::WorldPtr           world_;             // A pointer to the gazebo world.
        physics::ModelPtr           robot_model_;       // Pointer to the model
        physics::ModelPtr           ball_model_;    // Pointer to the football
        physics::LinkPtr            ball_link_;     //Pointer to the football link

        ros::NodeHandle*            rosnode_;           // A pointer to the ROS node.
        ros::Subscriber             ModelStates_sub_;
        ros::Subscriber             Velcmd_sub_;
        ros::Subscriber             CoachInfo_sub_;

        ros::Subscriber             cyan_sendingoff_;
        ros::Subscriber             magenta_sendingoff_;
        ros::Subscriber             actioncmd_sub_;     //listen the action command
        ros::Publisher              omin_vision_pub_;      /* four publishers cooresponding to those in world_model.cpp */
        ros::Publisher              debug_pub_;
        ros::Publisher              Ballisholding_pub;

        ros::ServiceClient          dribbleId_client_;

        boost::thread               message_callback_queue_thread_;     // Thead object for the running callback Thread.
        boost::thread               service_callback_queue_thread_;
        boost::mutex                msgCB_lock_;        // A mutex to lock access to fields that are used in ROS message callbacks
        boost::mutex                srvCB_lock_;        // A mutex to lock access to fields that are used in ROS service callbacks
        ros::CallbackQueue          message_queue_;     // Custom Callback Queue. Details see http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning
        ros::CallbackQueue          service_queue_;     // Custom Callback Queue
        event::ConnectionPtr        update_connection_;         // Pointer to the update event connection

        gazebo_msgs::ModelStates    model_states_;          // Container for the ModelStates msg
        model_state                 robot_state_;
        model_state                 ball_state_;
        nubot_common::BallInfo        ball_info_;
        nubot_common::RobotInfo       teamate_info_;
        nubot_common::ObstaclesInfo   obstacles_info_;
        nubot_common::OminiVisionInfo omni_info_;
        nubot_common::BallIsHolding   ballisholding_info_;
        //common::Time                  receive_sim_time_;
        std_msgs::Float64MultiArray   debug_msgs_;

        math::Vector3d               desired_rot_vector_;
        math::Vector3d               desired_trans_vector_;
        math::Vector3d               nubot_ball_vec_;
        math::Vector3d               kick_vector_world_;
        math::Rand                  rand_;
        std::string                 robot_namespace_;   // robot namespace. Not used yet.
        std::string                 model_name_;
        std::string                 ball_name_;
        std::string                 ball_chassis_;
        std::string                 cyan_pre_;
        std::string                 mag_pre_;
        unsigned int                ball_index_;
        unsigned int                robot_index_;

        double                      nubot_ball_vec_len_;
        double                      dribble_distance_thres_;
        double                      dribble_angle_thres_;
        double                      Vx_cmd_;
        double                      Vy_cmd_;
        double                      w_cmd_;
        double                      force_;                     //kick ball force
        double                      dribble_P_;                 // P gain
        double                      dribble_I_;                 // I gain
        double                      dribble_D_;                 // D gain
        double                      I_term_max_;                // maximum I term
        double                      I_term_min_;                // minimum I term
        double                      field_length_;
        double                      field_width_;
        double                      angle_error_degree_;
        double                      noise_scale_;               // scale of gaussian noise
        double                      noise_rate_;                // how frequent the noise generates
        int                         mode_;                      //kick ball mode
        int                         nubot_num_;
        char                        match_mode_;

        unsigned int                model_count_;                // Number of models
        bool                        dribble_req_;
        bool                        shot_req_;
        bool                        ModelStatesCB_flag_;         // Indicate receiving messages
        bool                        judge_nubot_stuck_;          // decide when to judge
        bool                        is_kick_;
        bool                        is_dribble_;                // flag to indicate dribbling
        bool                        flip_cord_;                 // flip the coordinate frame
        bool                        can_move_;

        int                         AgentID_;

        nubot_state                 state_;
        nubot_substate              sub_state_;
        Obstacles                   *obs_;
        dynamic_reconfigure::Server<nubot_gazebo::NubotGazeboConfig> *reconfigureServer_;

        /// \brief ModelStates message CallBack function
        /// \param[in] _msg model_states msg shared pointer
        void model_states_CB(const gazebo_msgs::ModelStates::ConstPtr& _msg);
        void SendingOff_CB(const nubot_common::SendingOff::ConstPtr & _msg);
        /// \brief VelCmd message CallBack function
        /// \param[in] cmd VelCmd msg shared pointer
        void vel_cmd_CB(const nubot_common::VelCmd::ConstPtr& cmd);

        /// \brief coach info CallBack function
        /// \param[in] cmd coach info msg shared pointer
        void coachinfo_CB(const nubot_common::CoachInfo::ConstPtr& cmd);

        /// \brief action command CallBack function
        /// \param[in] action command msg shared pointer
        void actionCmd_CB(const nubot_common::ActionCmd::ConstPtr& actioncmd);

        /// \brief Custom message callback queue thread
        void message_queue_thread();

        /// \brief Custom service callback queue thread
        void service_queue_thread();

        /// \brief Updating models' states
        /// \return 1: updating model info success 0: not success
        bool update_model_info(void);

        /// \brief Nubot moving fuction: rotation + translation
        /// \param[in] linear_vel_vector translation velocity 3D vector
        /// \param[in] angular_vel_vector rotation velocity 3D vector
        void nubot_locomotion(math::Vector3d linear_vel_vector, math::Vector3d angular_vel_vector);

        /// \brief Nubot dribbling ball function. The football follows nubot movement.
        void dribble_ball(void);

        /// \brief Nubot kicking ball
        /// \param[in] mode kick ball mode FLY or RUN
        /// \param[in] vel initial velocity of the ball kicked; used in RUN mode; not used in FLY mode
        void kick_ball(int mode, double vel);

        /// \brief  Get the value of flag is_hold_ball_
        /// \return 1: is holding ball 0: is not holding ball
        bool get_is_hold_ball(void);

        /// \brief Determine whether nubot stuck or not.
        /// \return 1: stuck 0: not stuck
        bool get_nubot_stuck(void);

        /// \brief Publish messages to world_model node
        void message_publish(void);

        /// \brief Robot action controlled by real-robot code. Need to connect to coach.
        void nubot_be_control(void);

        /// \brief For testing seprate functions
        void nubot_test(void);

        /// \brief Dynmaic recofigure calback function
        void config(nubot_gazebo::NubotGazeboConfig &config, uint32_t level);

        /// \brief in real world, this flag is determined by the status of power; however, in
        /// simulation, if the robot beyond the green border(carpet), it is not valid.
        /// \param[in]  x,y  x and y component of robot's position
        /// \param[out] if robot is valid, return true, otherwise return false
        bool is_robot_valid(double x, double y);

        /// \brief return a value representing real world noise. The noise complies with Normal(gaussian) distribution
        /// with mean 0 and derivation 1, that is noise~N(0,1)
        /// \param[in] scale            the magnitude of the noise is within [0, scale]
        /// \param[in] probability      the probability of generating noise, probability should be in [0,1]
        double  noise(double scale, double probability=0.01);

        /// \brief return the velocity after limitting the acceleration of nubot model
        /// \brief we limit the acceleration of every wheel for better simulation of real match
        /// \param[in] duration the duration between two commands of velocity
        /// \param[in] model_vel the real linear velocity of model at the last step
        /// \param[in] target_vel the target linear velocity of model now
        /// \param[in] model_vel the real angular velocity of model at the last step
        /// \param[in] target_vel the target angular velocity of model now
        math::Vector3d accelerateLimit(double duration, math::Vector3d  model_linear_vel, math::Vector3d target_linear_vel,math::Vector3d  model_ang_vel, math::Vector3d target_ang_vel);

        /// \brief return the velocity after limitting the speed of nubot model
        /// \brief we limit the speed of every wheel for better simulation of real match
        /// \param[in] target_vel the target linear velocity of model now
        math::Vector3d speedLimit( math::Vector3d target_linear_vel,math::Vector3d target_ang_vel);

    public:
        /// \brief Constructor. Will be called firstly
        NubotGazebo();

        /// \brief Destructor
        virtual ~NubotGazebo();

    protected:
        /// \brief Load the controller.
        /// Required by model plugin. Will be called secondly
        void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) ;

        /// \brief Update the controller. It is running every simulation iteration.
        /// [realtime factor] = [realtime update rate] * [max step size].
        virtual void update_child();

        /// \brief Model Reset function.
        /// Not required by model plugin. It is triggered when the world resets.
        virtual void Reset();
  };
}

#endif //! NUBOT_GAZEBO_HH
