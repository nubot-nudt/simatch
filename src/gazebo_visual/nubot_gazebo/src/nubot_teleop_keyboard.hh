#ifndef NUBOT_TELEOP_KEYBOARD_HH_
#define NUBOT_TELEOP_KEYBOARD_HH_

#include <ros/ros.h>
#include <string>

#include <nubot_common/VelCmd.h>
#include <nubot_common/BallHandle.h>
#include <nubot_common/Shoot.h>


namespace nubot
{
    /// \class NubotTeleopKey
    /// \brief Teleoperate nubot using keyboad.
    class NubotTeleopKey
    {
        public:

            /// \brief Constructor
            NubotTeleopKey();

            /// \brief Service client for ball-dribbling
            ros::ServiceClient ballhandle_client_;

            /// \brief Service client for ball-shooting
            ros::ServiceClient shoot_client_;

            /// \brief Update function that receives keyboard input and acts accordingly
            void keyLoop();

        private:

            /// \brief Node handler
            ros::NodeHandle nh_;

            /// \brief ROS Publisher that publishes velocity messages to control robot locomotion
            ros::Publisher vel_pub;

            /// \brief Velocity messages
            nubot_common::VelCmd vel_cmd_;

            double vx_, vy_, w_;

            /// \brief Ball-shooting mode: ground pass or flip shot.
            int mode_;

            /// \brief Flags to indicate ball-dribbling
            bool dribble_flag_;

            /// \brief Flags to indicate ball-shooting
            double shot_flag_;

            /// \brief The last dribble flag
            bool last_dribble_flag_;

            /// \brief The prefix of robot name defined in .yaml file.
            std::string robot_prefix_;
    };
}

#endif // ! NUBOT_TELEOP_KEYBOARD_HH_
