#ifndef BALL_GAZEBO_HH
#define BALL_GAZEBO_HH

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>             // the core gazebo header files, including gazebo/math/gzmath.hh
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
// #include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo_msgs/ContactState.h>
#include <gazebo_msgs/ContactsState.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#include "nubot/core/core.hpp"


namespace gazebo{

  class BallGazebo : public ModelPlugin
  {
    private:

        physics::WorldPtr           world_;             // A pointer to the gazebo world.
        physics::ModelPtr           football_model_;       // Pointer to the model

        ros::NodeHandle*            rosnode_;           // A pointer to the ROS node.
        ros::Subscriber             joy_sub_;
        event::ConnectionPtr        update_connection_;         // Pointer to the update event connection
        std::string                 football_chassis_;
        physics::LinkPtr            football_link_;     //Pointer to the football link

        double                      vel_x_;
        double                      vel_y_;
        double                      mu_;                // frictional coefficient
        double                      field_length_;
        double                      field_width_;

        /// \brief joystick callback function
        void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

        /// \brief a work-around for rolling frction
        /// \param[in] mu   --  friction coefficient
        void ball_vel_decay(double mu);

        /// \brief Detect whether ball is out of the field and put it in a specific position
        void detect_ball_out(void);

    public:
        /// \brief Constructor. Will be called firstly
        BallGazebo();

        /// \brief Destructor
        virtual ~BallGazebo();

    protected:
        /// \brief Load the controller.
        /// Required by model plugin. Will be called secondly
        void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);

        /// \brief Update the controller. It is running every simulation iteration.
        /// [realtime factor] = [realtime update rate] * [max step size].
        virtual void UpdateChild();
  };
}

#endif //! BALL_GAZEBO_HH
