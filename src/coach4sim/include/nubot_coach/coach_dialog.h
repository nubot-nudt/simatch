#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>
#include <QString>
#include <QDir>
#include <QCloseEvent>
#include <QHostAddress>
#include <QTime>
#include <QTimer>
#include <QVector>
#include <QRadioButton>
#include <QKeyEvent>
#include <QWaitCondition>
#include <QPixmap>
#include <QtNetwork/QTcpServer>
#include <QtNetwork/QTcpSocket>
#include <ui_coach_dialog.h>
#include <robot2coach.h>
#include <json_parse.h>
#include <coach2refbox.h>
#include <QMessageBox>
#include <qgraphicsscene.h>
#include <qgraphicsview.h>
#include <QGraphicsPixmapItem>
#include <QGraphicsLineItem>
#include <QGraphicsTextItem>

#define  WIDTH  700/1800                                                    //场地缩放单位长度
#define  HEIGHT 467/1200

namespace Ui {
class Dialog;
}

class Dialog : public QDialog
{
    Q_OBJECT

public:
    Dialog(nubot::Robot2coach_info & robot2coach,nubot::MessageFromCoach & coach2robot,QWidget *parent = 0);
    ~Dialog();
    void paintEvent(QPaintEvent *event);
    void keyPressEvent(QKeyEvent *event);
    void closeEvent(QCloseEvent *event);

    void showRobot_info_();                       //显示机器人策略信息
    void showAll_info_();                         //显示所有机器人和球的位置和速度信息
    int  ballPos_fuse();                          //在多个机器人都看到球的情况下做融合,仅用于绘图,返回机器人编号
    void RefBox_Info();                           //裁判盒发指令
    void disableButton_();                        //连接裁判盒后，按钮失效
    void enableButton_();                         //断开裁判盒过后，启用所有按键

    void buttonDelay_();                          //按钮按下时的时延
    void restItems_();                            //重置items位置

public:
    nubot::Robot2coach_info * robot2coach_info_;        //用于存放机器人上传的信息
    nubot::MessageFromCoach * coach2robot_info_;        //用于存放coach下发的信息
    nubot::JSONparse *json_parse_;                      //解析json
    nubot::Coach2refbox *coach2refebox_;                //打包机器人信息准备上传

    QPixmap field_img_init_;                       //场地图像
    QPixmap field_img_home_;                       //奥拓楼场地图像
    QPixmap robot_img_[OUR_TEAM];                  //机器人图像
    QPixmap ball_img_;                             //球图像
    QPixmap obs_img_[OUR_TEAM];                              //障碍物图像

    QGraphicsScene *scene_;
    QGraphicsPixmapItem *field_;                   //field item
    QGraphicsPixmapItem *ball_;                    //ball item
    QGraphicsPixmapItem *robot_[OUR_TEAM];         //robot item
    QGraphicsPixmapItem *obstacle_[OUR_TEAM][MAX_OBSNUMBER_CONST];      //obstacles item
    QGraphicsLineItem *velocity_;                  //velocity item
    QGraphicsTextItem *role_[ROLENUM];

    QTcpSocket *tcpSocket_;
    QHash<QTcpSocket*, QByteArray*> buffers;
    QHash<QTcpSocket*, qint32*> sizes;

    //用于绘图和显示的简化数据
    nubot::DPoint2s _robot_pos[OUR_TEAM];
    nubot::DPoint2s _robot_vel[OUR_TEAM];
    short _robot_ori[OUR_TEAM];
    nubot::DPoint2s _ball_pos[OUR_TEAM];
    nubot::DPoint2s _ball_vel[OUR_TEAM];
    nubot::DPoint2s _obstacles[OUR_TEAM][MAX_OBSNUMBER_CONST];

private:
    Ui::Dialog *ui;
    int display_choice_;                              //用于显示的选择
    int score_cyan_;
    int score_magenta_;

    bool isObs_display_;                              //判断是否显示障碍物以及对手
    bool isConnect_RefBox_;
    bool isUpload_worldmodel_;                        //是否上传worldmodel
    bool teamflag_;                                   //队伍标志位　　0 MAGENTA(默认),1 CYAN
    bool isAtHome_;                                   //在家里调试，场地图不一样
    int  groundflag_;                                 //场地方向标志  1 left2right（默认），-1 right2left

    QString infoShow_[OUR_TEAM];                             //用于显示机器人策略信息
    QString allShow_[OUR_TEAM];                             //用于显示所有机器人和球的位置和速度信息
    QString allShow_combine_;

private slots:
    void timerUpdate();                              //定时器槽函数
    void OnReceive_();                               //裁判盒消息接收
    void displayError_(QAbstractSocket::SocketError socketError);    //返回连接错误

    void on_radioButton_clicked();                   //radiobutton槽函数,用于选择绘图的机器人编号
    void on_radioButton_1_clicked();
    void on_radioButton_2_clicked();
    void on_radioButton_3_clicked();
    void on_radioButton_4_clicked();
    void on_radioButton_5_clicked();

    void on_Score1_up_clicked();                     //分数控制槽函数
    void on_Score1_down_clicked();
    void on_Score0_up_clicked();
    void on_Score0_down_clicked();

    void on_startButton_clicked();                   //控制面板的槽函数
    void on_stopButton_clicked();
    void on_kickoff_clicked();
    void on_kickoff_opp_clicked();
    void on_penalty_clicked();
    void on_penalty_opp_clicked();
    void on_corner_clicked();
    void on_corner_opp_clicked();
    void on_throwin_clicked();
    void on_throwin_opp_clicked();
    void on_freekick_clicked();
    void on_freekick_opp_clicked();
    void on_goalkick_clicked();
    void on_goalkick_opp_clicked();
    void on_dropball_clicked();
    void on_park_clicked();
#if 0
    void on_test_mode_clicked();
    void on_test_stop_clicked();

    void on_location_test_clicked();               //测试模式选择
    void on_circle_test_clicked();
    void on_move_mode_clicked();
    void on_pass_mode_clicked();
    void on_catch_mode_clicked();
    void on_shoot_mode_clicked();
#endif
    void on_cyan_clicked();                         //队伍选择
    void on_magenta_clicked();

    void on_connectRefe_clicked();                  //链接裁判盒
    void on_upload_clicked();                       //上传

    void on_change_ground_clicked();                //场地方向选择
    void on_field_home_clicked();                   //场地大小选择
    void on_obstacles_clicked();                    //是否显示障碍物
};


#endif // DIALOG_H
