#include <packet2Refbox.h>

using namespace nubot;


Packet2refbox::Packet2refbox()
{
    jsonObject_=new QJsonObject;
    mPacket_.type_=QString("worldstate");
    mPacket_.teamName_= QString("NuBot");
    mPacket_.globalIntention_= QString();
    mPacket_.robots_.clear();
    mPacket_.balls_.clear();
    mPacket_.obstacles_.clear();
}

Packet2refbox::~Packet2refbox()
{
    cleanupJSONObject();
}

int Packet2refbox::getSize()
{
    generateJSON();

    return jsonObject_->size();
}

//Team level setters
void Packet2refbox::setType(const QString type)
{
    mPacket_.type_=type;
}

void Packet2refbox::setTeamIntention(const QString intention)
{
    mPacket_.globalIntention_=intention;
}

//Robot level setters
void Packet2refbox::setRobotPose(const quint8 robotId, const DPoint2f pose, const float orien)
{
    int index = 0;
    bool isFound = false;

    try
    {
        isRobotPresent(robotId,isFound,index);
        if(!isFound)
        {
            addRobot(robotId);
            isRobotPresent(robotId,isFound,index);
        }
        mPacket_.robots_.at(index).pose=pose;
        mPacket_.robots_.at(index).orien=orien;
    }
    catch(exception &e)
    {
        throw e;
    }
}

void Packet2refbox::setRobotTargetPose(const quint8 robotId, const DPoint2f targetpose)
{
    int index = 0;
    bool isFound = false;

    try
    {
        isRobotPresent(robotId,isFound,index);
        if(!isFound)
        {
            addRobot(robotId);
            isRobotPresent(robotId,isFound,index);
        }
        mPacket_.robots_.at(index).targetPose=targetpose;
    }
    catch(exception &e)
    {
        throw e;
    }
}

void Packet2refbox::setRobotVelocity(const quint8 robotId, const DPoint2f velocity)
{
    int index = 0;
    bool isFound = false;

    try
    {
        isRobotPresent(robotId, isFound, index);
        if(!isFound)
        {
            addRobot(robotId);
            isRobotPresent(robotId, isFound, index);
        }

        mPacket_.robots_.at(index).velocity = velocity;
    }
    catch (exception &e)
    {
        throw e;
    }
}

void Packet2refbox::setRobotIntention(const quint8 robotId, const QString intention)
{
    int index = 0;
    bool isFound = false;

    try
    {
        isRobotPresent(robotId, isFound, index);
        if(!isFound)
        {
            addRobot(robotId);
            isRobotPresent(robotId, isFound, index);
        }

        mPacket_.robots_.at(index).intention = intention;
    }
    catch (exception &e)
    {
        throw e;
    }
}

void Packet2refbox::setRobotBatteryLevel(const quint8 robotId, const float level)
{
    int index = 0;
    bool isFound = false;

    try
    {
        isRobotPresent(robotId, isFound, index);
        if(!isFound)
        {
            addRobot(robotId);
            isRobotPresent(robotId, isFound, index);
        }

        mPacket_.robots_.at(index).batteryLevel = level;
    }
    catch (exception &e)
    {
        throw e;
    }
}

void Packet2refbox::setRobotBallPossession(const quint8 robotId, const bool hasBall)
{
    int index = 0;
    bool isFound = false;

    try
    {
        isRobotPresent(robotId, isFound, index);
        if(!isFound)
        {
            addRobot(robotId);
            isRobotPresent(robotId, isFound, index);
        }

        mPacket_.robots_.at(index).hasBall = hasBall;
    }
    catch (exception &e)
    {
        throw e;
    }
}

//Ball setters
void Packet2refbox::addBall(const DPoint2f position, const DPoint2f velocity, const float confidence)
{
    ballStructure ball;

    ball.position = position;
    ball.velocity = velocity;
    ball.confidence = confidence;

    mPacket_.balls_.push_back(ball);
}

//Obstacle setters
void Packet2refbox::addObstacle(const DPoint2f position, const DPoint2f velocity)
{
    obstacleStructure obstacle;

    obstacle.position = position;
    obstacle.velocity = velocity;
    obstacle.radius = 0.25;
    obstacle.confidence = 0.5;

    mPacket_.obstacles_.push_back(obstacle);
}

//Global setters
void Packet2refbox::setAgeMilliseconds(const double age)
{
    mPacket_.age_=age;
}

//Robot functions
void Packet2refbox::isRobotPresent(const quint8 robotId, bool &isPresent, int &index)
{
    robotList::iterator robotIter = mPacket_.robots_.begin();
    isPresent = false;
    index = 0;

    try
    {
        for(robotIter = mPacket_.robots_.begin(); (robotIter < mPacket_.robots_.end()) && (!isPresent); robotIter++)
        {
            if(robotIter->robotId == robotId)
            {
                isPresent = true;
                index = robotIter-mPacket_.robots_.begin();
            }
        }
    }
    catch (exception &e)
    {
        throw e;
    }
}

void Packet2refbox::addRobot(const quint8 robotId)
{
    robotStructure robot;

    try
    {
        robot.robotId = robotId;
        robot.pose = DPoint2f(0,0);
        robot.orien =0;
        robot.velocity = DPoint2f(0,0);
        robot.targetPose = DPoint2f(0,0);
        robot.intention = QString();
        robot.batteryLevel = 0.0;
        robot.hasBall = false;

        mPacket_.robots_.push_back(robot);
    }
    catch (exception &e)
    {
        throw e;
    }
}

void Packet2refbox::cleanupJSONObject()
{
    QStringList keylist=jsonObject_->keys();
    for(int i=0;i<keylist.length();i++)
        jsonObject_->remove(keylist.at(i));
    if(!jsonObject_->isEmpty())
        qDebug()<<"JSON is not empty";
}

//JSON functions
void Packet2refbox::generateJSON()
{
    try
    {
        cleanupJSONObject();

        //Add type string
        QJsonValue type=mPacket_.type_;
        jsonObject_->insert("type",type);

        //Add team name string
        QJsonValue teamName=mPacket_.teamName_;
        jsonObject_->insert("teamName",teamName);

        //Add team intention
        QJsonValue teamIntention=mPacket_.globalIntention_;
        jsonObject_->insert("intention",teamIntention);

        //Adding robots
        QJsonArray robots;
        addRobotsJSON(robots);
        jsonObject_->insert("robots",robots);

        //Adding balls
        QJsonArray balls;
        addBallsJSON(balls);
        jsonObject_->insert("balls",balls);

        //Adding obstacles
        QJsonArray obstacles;
        addObstaclesJSON(obstacles);
        jsonObject_->insert("obstacles",obstacles);

        //Adding ageMs
        QJsonValue ageMs=mPacket_.age_;
        jsonObject_->insert("ageMs",ageMs);
    }
    catch(exception &e)
    {
        throw e;
    }
}

void Packet2refbox::addRobotsJSON(QJsonArray &array)
{
    robotList::iterator robotIter = mPacket_.robots_.begin();
    try
    {
        for(robotIter; robotIter<mPacket_.robots_.end(); robotIter++)
        {
            QJsonObject obj;
            //id
            QJsonValue robotId=robotIter->robotId;
            obj.insert("id",robotId);

            //pose
            QJsonArray pose;
            QJsonValue poseX=-robotIter->pose.y_;     //比赛规定的坐标跟我们有区别，纵坐标=横坐标，横坐标=-纵坐标
            QJsonValue poseY=robotIter->pose.x_;
            QJsonValue posePhi=robotIter->orien;
            /*if(robotIter->orien<M_PI/2)             //标准世界模型跟我们的x轴，y轴不同，但貌似角度并没有变化
                posePhi=robotIter->orien+M_PI/2;
            else
                posePhi=robotIter->orien-3*M_PI/2;*/
            pose.append(poseX);
            pose.append(poseY);
            pose.append(posePhi);
            obj.insert("pose",pose);

            //targetPose
            QJsonArray targetPose;
            QJsonValue targetPoseX=-robotIter->targetPose.y_;     //比赛规定的坐标跟我们有区别，纵坐标=横坐标，横坐标=-纵坐标
            QJsonValue targetPoseY=robotIter->targetPose.x_;
            targetPose.append(targetPoseX);
            targetPose.append(targetPoseY);
            targetPose.append(0);
            obj.insert("targetPose",targetPose);

            //velocity
            QJsonArray velocity;
            QJsonValue velocityX=-robotIter->velocity.y_;     //比赛规定的坐标跟我们有区别，纵坐标=横坐标，横坐标=-纵坐标
            QJsonValue velocityY=robotIter->velocity.x_;
            velocity.append(velocityX);
            velocity.append(velocityY);
            velocity.append(0);
            obj.insert("velocity",velocity);

            //intention
            QJsonValue robotIntention=robotIter->intention;
            obj.insert("intention",robotIntention);

            //battery level
            QJsonValue batteryLevel=robotIter->batteryLevel;
            obj.insert("batteryLevel",batteryLevel);

            //ball engaged
            QJsonValue ballEngaged=robotIter->hasBall;
            obj.insert("ballEngaged",ballEngaged);

            //add robot to array
            array.append(obj);
        }
    }
    catch(exception &e)
    {
        throw e;
    }
}

void Packet2refbox::addBallsJSON(QJsonArray &array)
{
    ballList::iterator ballIter=mPacket_.balls_.begin();
    try
    {
        for(ballIter; ballIter < mPacket_.balls_.end(); ballIter++)
        {
            QJsonObject obj;

            //position
            QJsonArray position;
            QJsonValue positionX=-ballIter->position.y_;     //比赛规定的坐标跟我们有区别，纵坐标=横坐标，横坐标=-纵坐标
            QJsonValue positionY=ballIter->position.x_;
            position.append(positionX);
            position.append(positionY);
            position.append(0);
            obj.insert("position",position);

            //velocity
            QJsonArray velocity;
            QJsonValue velocityX=-ballIter->velocity.y_;     //比赛规定的坐标跟我们有区别，纵坐标=横坐标，横坐标=-纵坐标
            QJsonValue velocityY=ballIter->velocity.x_;
            velocity.append(velocityX);
            velocity.append(velocityY);
            velocity.append(0);
            obj.insert("velocity",velocity);

            //confidence
            QJsonValue confidence=ballIter->confidence;
            obj.insert("confidence",confidence);

            //add ball to array
            array.append(obj);
        }
    }
    catch(exception &e)
    {
        throw e;
    }
}

void Packet2refbox::addObstaclesJSON(QJsonArray &array)
{
    obstacleList::iterator obstacleIter=mPacket_.obstacles_.begin();

    try
    {
        for(obstacleIter; obstacleIter<mPacket_.obstacles_.end(); obstacleIter++)
        {
            QJsonObject obj;

            //position
            QJsonArray position;
            QJsonValue positionX=-obstacleIter->position.y_;     //比赛规定的坐标跟我们有区别，纵坐标=横坐标，横坐标=-纵坐标
            QJsonValue positionY=obstacleIter->position.x_;
            position.append(positionX);
            position.append(positionY);
            position.append(0);
            obj.insert("position",position);

            //radius
            QJsonValue radius=obstacleIter->radius;
            obj.insert("radius",radius);  //我们你没有计算半径

            //velocity
            QJsonArray velocity;
            QJsonValue velocityX=-obstacleIter->velocity.y_;     //比赛规定的坐标跟我们有区别，纵坐标=横坐标，横坐标=-纵坐标
            QJsonValue velocityY=obstacleIter->velocity.x_;
            velocity.append(velocityX);
            velocity.append(velocityY);
            obj.insert("velocity",velocity);

            //confidence
            QJsonValue confidence=obstacleIter->confidence;
            obj.insert("confidence",confidence);

            //add obstacle to array
            array.append(obj);
        }
    }
    catch(exception &e)
    {
        throw e;
    }
}
