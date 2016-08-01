#include "ball_gazebo.hh"

#define idx_X 1
#define idx_Y 0
#define idx_w 3

#define Button_A    0
#define Button_B    1
#define Button_X    2
#define Button_Y    3

#define Button_Up   5
#define Button_Down 4

const double        g = 9.8;                    // gravity coefficient
const double        m = 0.41;                   // ball mass (kg)

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(BallGazebo)

BallGazebo::BallGazebo()
{}

BallGazebo::~BallGazebo()
{}

void BallGazebo::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
{
    world_ = _parent->GetWorld();
    football_model_ = _parent;
    rosnode_ = new ros::NodeHandle();
    rosnode_->param("/football/chassis_link",  football_chassis_,   std::string("football::ball") );
    rosnode_->param("/field/length",           field_length_,      18.0);
    rosnode_->param("/field/width",            field_width_,       12.0);


    football_link_ = football_model_->GetLink(football_chassis_);
    if(!football_link_)
        ROS_ERROR("link [%s] does not exist!", football_chassis_.c_str());

    joy_sub_ =rosnode_->subscribe<sensor_msgs::Joy>("joy", 2, &BallGazebo::joyCallback, this);

    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&BallGazebo::UpdateChild, this));
}

void BallGazebo::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    /*速度指令*/
    vel_y_ = joy->axes[idx_X]*5;
    vel_x_ = -joy->axes[idx_Y]*5;
}

void BallGazebo::UpdateChild()
{
    static math::Vector3 ball_vel(0, 0, 0);

    detect_ball_out();
    if(std::sqrt(vel_x_*vel_x_+vel_y_*vel_y_)>1)
    {
      ball_vel.Set(vel_x_, vel_y_, 0);
      football_model_->SetLinearVel(ball_vel);
    }
    rosnode_->param("/general/ball_decay_coef", mu_, 0.5);
    ball_vel_decay(mu_);
}

void BallGazebo::ball_vel_decay(double mu)
{
    math::Vector3   vel = football_model_->GetWorldLinearVel();
    static double   last_vel_len = vel.GetLength();
    double          vel_len = vel.GetLength();

    if(vel_len > 0.0)
    {
        if(football_model_->GetWorldPose().pos.z <= 0.12 &&
                !(last_vel_len - vel_len > 0) )     // when the ball is not in the air && when
                                                    // it does not decelerate anymore
        {
            double force = -mu*m*g;
            football_link_->AddForce(vel.Normalize()*force);
        }
    }
    else if(vel_len <= 0.0)
    {
        vel_len = 0.0;
        football_model_->SetLinearVel(math::Vector3::Zero);
    }

    last_vel_len = vel_len;
}

void BallGazebo::detect_ball_out(void)
{
    double pos_x = football_model_->GetWorldPose().pos.x;
    double pos_y = football_model_->GetWorldPose().pos.y;
    int a = pos_x > 0? 1 : -1;
    int b = pos_y > 0? 1 : -1;

    if(fabs(pos_x)>field_length_/2.0)
    {
        math::Pose  target_pose( math::Vector3 (a*(field_length_/2.0-0.02), pos_y, 0.12), math::Quaternion(0,0,0) );
        football_model_->SetWorldPose(target_pose);
        football_model_->SetLinearVel(math::Vector3::Zero);
    }
    else if(fabs(pos_y) > field_width_/2.0)
    {
        math::Pose  target_pose( math::Vector3 (pos_x, b*(field_width_/2.0 - 0.02), 0.12), math::Quaternion(0,0,0) );
        football_model_->SetWorldPose(target_pose);
        football_model_->SetLinearVel(math::Vector3::Zero);
    }
}
