#include <coach2refbox.h>

using namespace nubot;


Coach2refbox::Coach2refbox()
{
    for(int i=0;i<ACTION_NUM;i++)
        actions_[i]=" ";
    actions_[0]="Stucked";
    actions_[1]="Penalty";
    actions_[2]="CanNotSeeBall";
    actions_[3]="SeeNotDribbleBall";
    actions_[4]="TurnForShoot";
    actions_[5]="TurnForShoot_Robot";
    actions_[6]="AtShootSituation";
    actions_[7]="TurnToPass";
    actions_[8]="TurnToPass Robot";
    actions_[9]="StaticPass";
    actions_[10]="AvoidObs";
    actions_[11]="Catch Positioned";
    actions_[12]="Positioned";
    actions_[13]="Positioned Static";
    actions_[14]="KickCoop";
    actions_[15]="KickCoop turn";
    actions_[16]="CatchBall";
    actions_[17]="CatchBall slow";
    actions_[18]="CircleTest";
    actions_[19]="MoveWithBall";
    actions_[20]="TeleopJoy";
    actions_[21]="No Action";
}

Coach2refbox::~Coach2refbox()
{

}

void Coach2refbox::packWorldmodel_(Robot2coach_info *robot2coach_info_)
{
    cleanPacket_();

    static  std::ofstream json_output("/home/nubot8/json.txt");

    //简化上传数据
    for(int i=0;i<OUR_TEAM;i++)
    {
        _robot_pos[i]=DPoint2f(robot2coach_info_->RobotInfo_[i].getLocation().x_/100,robot2coach_info_->RobotInfo_[i].getLocation().y_/100);
        _robot_vel[i]=DPoint2f(robot2coach_info_->RobotInfo_[i].getVelocity().x_/100,robot2coach_info_->RobotInfo_[i].getVelocity().y_/100);
        _robot_target[i]=DPoint2f(robot2coach_info_->RobotInfo_[i].getTarget().x_/100,robot2coach_info_->RobotInfo_[i].getTarget().y_/100);
        _robot_ori[i]=robot2coach_info_->RobotInfo_[i].getHead().radian_;
        _ball_pos[i]=DPoint2f(robot2coach_info_->BallInfo_[i].getGlobalLocation().x_/100,robot2coach_info_->BallInfo_[i].getGlobalLocation().y_/100);
        _ball_vel[i]=DPoint2f(robot2coach_info_->BallInfo_[i].getVelocity().x_/100,robot2coach_info_->BallInfo_[i].getVelocity().y_/100);
    }
    for(int i=0;i<robot2coach_info_->Opponents_.size();i++)
        _obstacles[i]=DPoint2f(robot2coach_info_->Opponents_[i].x_/100,robot2coach_info_->Opponents_[i].y_/100);

    //Team level setters
    nubotpacket_.setTeamIntention(QString("active"));

    //Robot level setters
    for(int i=0;i<OUR_TEAM;i++)
    {
        if(robot2coach_info_->RobotInfo_[i].isValid())
        {
            nubotpacket_.addRobot(i+1);
            nubotpacket_.setRobotPose(i+1,_robot_pos[i],_robot_ori[i]);
            nubotpacket_.setRobotVelocity(i+1,_robot_vel[i]);
            nubotpacket_.setRobotTargetPose(i+1,_robot_target[i]);
            if(robot2coach_info_->RobotInfo_[i].getCurrentAction()<ACTION_NUM)              //防止数组越界
                nubotpacket_.setRobotIntention(i+1,actions_[robot2coach_info_->RobotInfo_[i].getCurrentAction()]);
            nubotpacket_.setRobotBatteryLevel(i+1,0.5);
            nubotpacket_.setRobotBallPossession(i+1,robot2coach_info_->RobotInfo_[i].getDribbleState());
        }
    }

    //Ball setters
    for(int i=0;i<OUR_TEAM;i++)
    {
        if(robot2coach_info_->BallInfo_[i].isLocationKnown())
        {
            short distance=_robot_pos[i].distance(_ball_pos[i]);
            nubotpacket_.addBall(_ball_pos[i],_ball_vel[i],0.001*(1000-distance));
        }
    }

    //Obstacles setters
    for(int i=0;i<robot2coach_info_->Opponents_.size();i++)
        nubotpacket_.addObstacle(_obstacles[i],DPoint2f(0,0));

    //ageMs setters
    nubotpacket_.setAgeMilliseconds(90);

    //update JSON
    jsonSize=nubotpacket_.getSize();
    QJsonDocument document;
    document.setObject(*nubotpacket_.jsonObject_);
    upload_array_ = document.toJson(QJsonDocument::Compact);            //把这个用tcpip传上去
    upload_array_.append('\0');
//    std::string json=upload_array_.data();
//    json_output<<json;
}

void Coach2refbox::cleanPacket_()
{
    nubotpacket_.mPacket_.balls_.clear();
    nubotpacket_.mPacket_.robots_.clear();
    nubotpacket_.mPacket_.obstacles_.clear();
}
