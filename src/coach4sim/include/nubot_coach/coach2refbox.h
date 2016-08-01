#ifndef COACH2REFBOX_H
#define COACH2REFBOX_H

#include <packet2Refbox.h>

using namespace std;
namespace nubot {

class Coach2refbox
{
public:
    Coach2refbox();
    ~Coach2refbox();

    void packWorldmodel_(Robot2coach_info *robot2coach_info_);
    void cleanPacket_();

public:
    Packet2refbox nubotpacket_;
    QString actions_[13];
    QByteArray upload_array_;            //上传的数据
    int  jsonSize;

    //简化换算上传数据
    DPoint2f _robot_pos[OUR_TEAM];
    DPoint2f _robot_vel[OUR_TEAM];
    DPoint2f _robot_target[OUR_TEAM];
    float _robot_ori[OUR_TEAM];
    DPoint2f _ball_pos[OUR_TEAM];
    DPoint2f _ball_vel[OUR_TEAM];
    DPoint2f _obstacles[MAX_OBSNUMBER_CONST*2];
};
}

#endif // COACH2REFBOX_H
