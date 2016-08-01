#include "coach_dialog.h"
#include <QIcon>
#include <QDebug>
#include <robot2coach.h>
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QString myDir=QCoreApplication::applicationDirPath();
    QDir::setCurrent(myDir);                                                     //改变当前目录到程序目录
    a.setWindowIcon(QIcon("../../../src/nubot/nubot_coach/source/app.png"));     //改变应用程序图标

    //system("sudo -S busybox route add -net 230.0.0.1 netmask 255.255.255.255 dev eth0"); //把230.0.0.1加入路由表
    /** 初始化ROS */
    ros::init(argc,argv,"nubot_coach_node");
    ros::Time::init();
    ROS_INFO("start coach process");

    //ros::NodeHandle node;
    nubot::Robot2coach robot2coach(argv);
    Dialog w(robot2coach.robot2coach_info, robot2coach.coach2robot_info);
    w.show();

    robot2coach.start();
    return a.exec();
}
