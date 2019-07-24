#ifndef NUBOT_TELEOP_KEYBOARD_HH_
#define NUBOT_TELEOP_KEYBOARD_HH_

#include <ros/ros.h>
#include <string>

#include <nubot_common/VelCmd.h>
#include <nubot_common/ActionCmd.h>
#include <nubot_common/BallIsHolding.h>


namespace nubot
{
    /// \class NubotTeleopKey
    /// \brief Teleoperate nubot using keyboad.
    class NubotTeleopKey
    {
        public:

            /// \brief Constructor
            NubotTeleopKey();

            /// \brief subscriber for ball-dribbling
            ros::Subscriber ballisholding_sub;


            /// \brief call back function for ballisholding
            void ballisholding_CB(const nubot_common::BallIsHolding::ConstPtr &ballisholding_info);

            nubot_common::BallIsHolding ballIsHolding_info_;
            /// \brief Update function that receives keyboard input and acts accordingly
            void keyLoop();

        private:

            /// \brief Node handler
            ros::NodeHandle nh_;

            /// \brief ROS Publisher that publishes velocity messages to control robot locomotion
            ros::Publisher vel_pub;

            /// \brief publisher for action command
            ros::Publisher actioncmd_pub;

            /// \brief action command
            nubot_common::ActionCmd actioncmd_info_;

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
