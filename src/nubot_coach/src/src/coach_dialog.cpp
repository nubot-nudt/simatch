#include "coach_dialog.h"

Dialog::Dialog(nubot::Robot2coach_info & robot2coach, nubot::MessageFromCoach & coach2robot, QWidget *parent) :
    QDialog(parent)
{
    robot2coach_info_= & robot2coach;                                 //将指针指向值空间
    coach2robot_info_= & coach2robot;
    json_parse_=new nubot::JSONparse;                                 //用于解析json文件
    coach2refebox_=new nubot::Coach2refbox;                           //生成上传的json文件

    scene_=new QGraphicsScene();                                      //尝试采用新的绘图方法

    timer=new QTimer(this);
    tcpSocket_=new QTcpSocket(this);

    QByteArray *buffer = new QByteArray();
    qint32 *s = new qint32(0);
    buffers.insert(tcpSocket_, buffer);
    sizes.insert(tcpSocket_, s);

    connect(timer,SIGNAL(timeout()),this,SLOT(timerUpdate()));               //定时函数的槽函数
    //connect(tcpSocket_, SIGNAL(readyRead()), this, SLOT(OnReceive_()));    //tcp/ip通信时用到的槽函数
    connect(tcpSocket_, SIGNAL(error(QAbstractSocket::SocketError)), this, SLOT(displayError_(QAbstractSocket::SocketError)));

    ui=new Ui::Dialog;                                                       //部分ui的初始化
    ui->setupUi(this);
    ui->radioButton->setChecked(true);
    ui->staticpass1->setChecked(true);
    ui->display->setScene(scene_);

    QRegExp rx("((2[0-4]//d|25[0-5]|[01]?//d//d?)//.){3}(2[0-4]//d|25[0-5]|[01]?//d//d?)");   //设置ip输入格式
    QRegExpValidator v(rx, 0);

    ui->IP_in->setValidator(&v);
    ui->IP_in->setInputMask("000.00.0.0;0");
    ui->IP_in->setText("173.17.1.2");

    ui->agentA_ID->setText("1");
    ui->agentA_ID->setValidator(new QIntValidator(1,OUR_TEAM,ui->agentA_ID));           //设置输入范围
    ui->agentB_ID->setText("1");
    ui->agentB_ID->setValidator(new QIntValidator(1,OUR_TEAM,ui->agentA_ID));

    ui->pointAin_X->setText("0");
    ui->pointAin_X->setValidator(new QIntValidator(FIELD_XLINE7,FIELD_XLINE1,ui->pointAin_X));
    ui->pointBin_X->setText("0");
    ui->pointBin_X->setValidator(new QIntValidator(FIELD_XLINE7,FIELD_XLINE1,ui->pointBin_X));

    ui->pointAin_Y->setText("0");
    ui->pointAin_Y->setValidator(new QIntValidator(FIELD_YLINE6,FIELD_YLINE1,ui->pointAin_Y));
    ui->pointBin_Y->setText("0");
    ui->pointBin_Y->setValidator(new QIntValidator(FIELD_YLINE6,FIELD_YLINE1,ui->pointBin_Y));

    ui->angleAin->setText("0");
    ui->angleAin->setValidator(new QIntValidator(-180,180,ui->angleAin));
    ui->angleBin->setText("0");
    ui->angleBin->setValidator(new QIntValidator(-180,180,ui->angleBin));

    ui->circle_radius->setText("0");
    ui->circle_radius->setValidator(new QIntValidator(0,500,ui->circle_radius));
    ui->circle_vel->setText("0");
    ui->circle_vel->setValidator(new QIntValidator(-500,500,ui->circle_vel));

    ui->target_x->setText("0");
    ui->target_x->setValidator(new QIntValidator(FIELD_XLINE7,FIELD_XLINE1,ui->target_x));
    ui->target_y->setText("0");
    ui->target_y->setValidator(new QIntValidator(FIELD_YLINE6,FIELD_YLINE1,ui->target_y));

    ui->shoot_force->setText("0");
    ui->shoot_force->setValidator(new QIntValidator(0,50,ui->shoot_force));

    ui->maxvel->setText("450");
    ui->maxvel->setValidator(new QIntValidator(100,500,ui->maxvel));
    ui->maxw->setText("10");
    ui->maxw->setValidator(new QIntValidator(1,15,ui->maxw));

    ui->currentState->setText("STOP ROBOT");                                    //显示当前机器人状态
    ui->teststate_dis->setText("STOP ROBOT");

    coach2robot_info_->MatchMode=STOPROBOT;                                     //coach重要信息的初始化
    coach2robot_info_->MatchType=STOPROBOT;
    coach2robot_info_->TestMode=Test_Stop;

    timer->start(30);                                                            //定时函数，每30ms

    this->setFixedSize(1150,700);                                                //固定窗口大小

    field_img_init_.load(":/field.png");     //载入场地图像
    field_img_home_.load(":/field_home.png");
    robot_img_[0].load(":/NUM1.png");        //载入机器人图像
    robot_img_[1].load(":/NUM2.png");
    robot_img_[2].load(":/NUM3.png");
    robot_img_[3].load(":/NUM4.png");
    robot_img_[4].load(":/NUM5.png");
    obs_img_[0].load(":/obstacles_1.png");
    obs_img_[1].load(":/obstacles_2.png");
    obs_img_[2].load(":/obstacles_3.png");
    obs_img_[3].load(":/obstacles_4.png");
    obs_img_[4].load(":/obstacles_5.png");
    ball_img_.load(":/ball.png");

    for(int i=0;i<OUR_TEAM;i++)
    {
        robot_img_[i]=robot_img_[i].scaled(30,30);
        obs_img_[i]=obs_img_[i].scaled(30,30);
    }
    ball_img_=ball_img_.scaled(20,20);

    field_=scene_->addPixmap(field_img_init_);             //载入初始化球场
    scene_->setSceneRect(0,0,748,476);                     //固定显示区域

    for(int i=0;i<OUR_TEAM;i++)
        for(int j=0;j<MAX_OBSNUMBER_CONST;j++)
        {
            obstacle_[i][j]=scene_->addPixmap(obs_img_[i]);                 //载入障碍物
            obstacle_[i][j]->setPos(1100,1100);                               //初始位置放到(900,900),不出现在视野里
        }

    role_[0]=scene_->addText("GOALIE");                    //载入角色item
    role_[1]=scene_->addText("ACTIVE");
    role_[2]=scene_->addText("PASSIVE");
    role_[3]=scene_->addText("MIDFIELD");
    role_[4]=scene_->addText("ASSISTANT");

    for(int i=0;i<OUR_TEAM;i++)
    {
        robot_[i]=scene_->addPixmap(robot_img_[i]);        //载入机器人
        robot_[i]->setPos(1100,1100);
        role_[i]->setDefaultTextColor(QColor(255,200,0));
        role_[i]->setPos(1100,1100);
    }

    ball_=scene_->addPixmap(ball_img_);                    //载入球
    ball_->setPos(1100,1100);

    velocity_=scene_->addLine(1100,1100,1101,1101,QPen(Qt::red, 5));  //初始化速度曲线位置
    pass2catch_=scene_->addLine(1100,1100,1101,1101,QPen(Qt::blue, 5));  //初始化传球路径

    init_style_();                    //初始化样式
    //一系列的标志初始化
    isObs_display_=false;
    isConnect_RefBox_=false;

    display_choice_=0;
    isAtHome_=0;
    teamflag_=0;
    score_cyan_=0;
    score_magenta_=0;
    groundflag_=1;
}

Dialog::~Dialog()
{
    delete json_parse_;                                    //释放申请的内存空间
    delete coach2refebox_;
    delete scene_;
    delete timer;
    delete tcpSocket_;

    delete ui;                                             //不能同时关闭ui线程和ros线程
}

void Dialog::paintEvent(QPaintEvent *event)
{
    //根据选择画图
    restItems_();      //重置items
    if(display_choice_==0)
    {
        //绘制机器人
        for(int i=0;i<OUR_TEAM;i++)
        {
            if(robot2coach_info_->RobotInfo_[i].isValid())
            {
                robot_[i]->setPos(groundflag_*_robot_pos[i].x_*WIDTH+359,-groundflag_*_robot_pos[i].y_*HEIGHT+223);
                robot_[i]->setTransformOriginPoint(15,15);
                robot_[i]->setRotation((groundflag_-1)*90-_robot_ori[i]);
                if(robot2coach_info_->RobotInfo_[i].getCurrentRole()<5)
                    role_[robot2coach_info_->RobotInfo_[i].getCurrentRole()]->setPos(groundflag_*_robot_pos[i].x_*WIDTH+344,
                                                                                 -groundflag_*_robot_pos[i].y_*HEIGHT+200);
            }
        }


        //绘制球
        int num=ballPos_fuse();
        if(num)
            ball_->setPos(groundflag_*_ball_pos[num-1].x_*WIDTH+364,-groundflag_*_ball_pos[num-1].y_*HEIGHT+228);

        //绘制融合后的障碍物
        if(isObs_display_)
            for(int i=0;i<OUR_TEAM;i++)
                if(robot2coach_info_->RobotInfo_[i].isValid())     //存在任意一个机器人在场
                    for(int j=0;j<MAX_OBSNUMBER_CONST;j++)
                        obstacle_[i][j]->setPos(groundflag_*_obstacles[i][j].x_*WIDTH+359,-groundflag_*_obstacles[i][j].y_*HEIGHT+223);

        //绘制传球路径(传球路径显示且有效)
        if(isP_C_display_ && robot2coach_info_->isPass_valid_&&
           robot2coach_info_->Catch_id_>0 && robot2coach_info_->Catch_id_<OUR_TEAM && robot2coach_info_->Pass_id_>0 && robot2coach_info_->Pass_id_<OUR_TEAM)
            pass2catch_->setLine(groundflag_*_robot_pos[robot2coach_info_->Catch_id_].x_*WIDTH+359,-groundflag_*_robot_pos[robot2coach_info_->Catch_id_].y_*HEIGHT+223,
                                 groundflag_*_robot_pos[robot2coach_info_->Pass_id_].x_*WIDTH+359,-groundflag_*_robot_pos[robot2coach_info_->Pass_id_].y_*HEIGHT+223);

    }
    else
    {
        if(robot2coach_info_->RobotInfo_[display_choice_-1].isValid())
        {
            //绘制机器人
            robot_[display_choice_-1]->setPos(groundflag_*_robot_pos[display_choice_-1].x_*WIDTH+359,-groundflag_*_robot_pos[display_choice_-1].y_*HEIGHT+223);
            robot_[display_choice_-1]->setTransformOriginPoint(15,15);
            robot_[display_choice_-1]->setRotation((groundflag_-1)*90-_robot_ori[display_choice_-1]);

            //绘制球
            if(robot2coach_info_->BallInfo_[display_choice_-1].isLocationKnown())
            {
                ball_->setPos(groundflag_*_ball_pos[display_choice_-1].x_*WIDTH+364,-groundflag_*_ball_pos[display_choice_-1].y_*HEIGHT+228);

                //绘制当前机器人识别的球速
                velocity_->setLine(groundflag_*_ball_pos[display_choice_-1].x_*WIDTH+374,-groundflag_*_ball_pos[display_choice_-1].y_*HEIGHT+238,
                                   groundflag_*(_ball_pos[display_choice_-1].x_*WIDTH+_ball_vel[display_choice_-1].x_)+374,
                                   -groundflag_*(_ball_pos[display_choice_-1].y_*HEIGHT+_ball_vel[display_choice_-1].y_)+238);
            }
            //绘制当前机器人识别的障碍物
            if(isObs_display_)
                for(int j=0;j<MAX_OBSNUMBER_CONST;j++)
                    obstacle_[display_choice_-1][j]->setPos(groundflag_*_obstacles[display_choice_-1][j].x_*WIDTH+359,
                                                            -groundflag_*_obstacles[display_choice_-1][j].y_*HEIGHT+223);
            if(ui->no_clear->isChecked())
                for(int j=0;j<static_obs.size();j++)
                    if(j<MAX_OBSNUMBER_CONST)
                        obstacle_[display_choice_-1][j]->setPos(groundflag_*static_obs[j].x_*WIDTH+359,
                                                                -groundflag_*static_obs[j].y_*HEIGHT+223);
        }
    }
}

void Dialog::keyPressEvent(QKeyEvent *event)               //保证安全，任何模式下空格都能发stop命令到机器人
{
    if(event->key()==Qt::Key_Space)
    {
         coach2robot_info_->MatchMode=STOPROBOT;
         ui->currentState->setText("STOP ROBOT");
         ui->teststate_dis->setText("STOP ROBOT");
    }
}

void Dialog::closeEvent(QCloseEvent *event)                //结束时关闭Ros进程
{

}

void Dialog::timerUpdate()
{
    //只要不是test模式,将最大速度,最大转速以及静态站位的模式下发
    if(coach2robot_info_->MatchMode!=TEST)
    {
        if(ui->staticpass1->isChecked())
            coach2robot_info_->id_A=1;
        else if(ui->staticpass2->isChecked())
            coach2robot_info_->id_A=2;

        coach2robot_info_->angleA=ui->maxvel->text().toShort();
        coach2robot_info_->angleB=ui->maxw->text().toShort();
    }

    current_valid_robots=0;
    //简化数据结构
    for(int i=0;i<OUR_TEAM;i++)
    {
        if(robot2coach_info_->RobotInfo_[i].isValid())
        {
            _robot_pos[i]=robot2coach_info_->RobotInfo_[i].getLocation();
            _robot_vel[i]=robot2coach_info_->RobotInfo_[i].getVelocity();
            _robot_ori[i]=robot2coach_info_->RobotInfo_[i].getHead().degree();
            _ball_pos[i]=robot2coach_info_->BallInfo_[i].getGlobalLocation();
            _ball_vel[i]=robot2coach_info_->BallInfo_[i].getVelocity();
            for(int j=0;j<MAX_OBSNUMBER_CONST;j++)
                _obstacles[i][j]=robot2coach_info_->Obstacles_[i][j];
            current_valid_robots+=1;
        }
        else
        {
            _robot_pos[i]=nubot::DPoint2s (0,0);
            _robot_vel[i]=nubot::DPoint2s (0,0);
            _robot_ori[i]=0;
            _ball_pos[i]=nubot::DPoint2s (0,0);
            _ball_vel[i]=nubot::DPoint2s (0,0);
            for(int j=0;j<MAX_OBSNUMBER_CONST;j++)
                _obstacles[i][j]=nubot::DPoint2s (0,0);
        }
    }

    if(ui->no_clear->isChecked()&&display_choice_!=0)
        for(int i=0;i<MAX_OBSNUMBER_CONST;i++)
        {
            if(static_obs.size()==0)
                static_obs.push_back(robot2coach_info_->Obstacles_[display_choice_-1][i]);
            else
            {
                for(int j=0;j<static_obs.size();j++)
                {
                    if(robot2coach_info_->Obstacles_[display_choice_-1][i].distance(static_obs[j])<200)
                        break;
                    if(j==static_obs.size()-1)
                        static_obs.push_back(robot2coach_info_->Obstacles_[display_choice_-1][i]);
                }
            }
        }

    update();                                             //重绘
    showAll_info_();
    if(isConnect_RefBox_)                                 //采用循环降低频率，不用重新写定时函数
    {
        static int count=0;
        if(count==15)
        {
            ui->text_disply->setStyleSheet("");                   //复位颜色
            count=0;
        }
        else
            count++;
    }
    if(isUpload_worldmodel_&&isConnect_RefBox_)              //采用循环降低频率，不用重新写定时函数
    {
        static int count1=0;
        if(count1==2)                                        //90ms一次
        {
            coach2refebox_->packWorldmodel_(robot2coach_info_);
            tcpSocket_->write(coach2refebox_->upload_array_);
            count1=0;
        }
        else
            count1++;
    }
}

//显示的选择
void Dialog::on_radioButton_clicked()
{
    display_choice_=0;
    velocity_->setLine(900,901,900,901);
}
void Dialog::on_radioButton_1_clicked()
{
    display_choice_=1;
}
void Dialog::on_radioButton_2_clicked()
{
    display_choice_=2;
}
void Dialog::on_radioButton_3_clicked()
{
    display_choice_=3;
}
void Dialog::on_radioButton_4_clicked()
{
    display_choice_=4;
}
void Dialog::on_radioButton_5_clicked()
{
    display_choice_=5;
}

//分数控制
void Dialog::on_Score1_up_clicked()
{
    score_cyan_=ui->Score1->value();
    score_cyan_++;
    ui->Score1->display(score_cyan_);
}
void Dialog::on_Score1_down_clicked()
{
    score_cyan_=ui->Score1->value();
    if(score_cyan_)
        score_cyan_--;
    ui->Score1->display(score_cyan_);
}
void Dialog::on_Score0_up_clicked()
{
    score_magenta_=ui->Score0->value();
    score_magenta_++;
    ui->Score0->display(score_magenta_);
}
void Dialog::on_Score0_down_clicked()
{
    score_magenta_=ui->Score0->value();
    if(score_magenta_)
        score_magenta_--;
    ui->Score0->display(score_magenta_);
}

//控制面板
void Dialog::on_startButton_clicked()
{
    coach2robot_info_->MatchMode=STARTROBOT;
    ui->currentState->setText("START ROBOT");
}

void Dialog::on_stopButton_clicked()
{
    coach2robot_info_->MatchMode=STOPROBOT;
    ui->currentState->setText("STOP ROBOT");
}

void Dialog::on_kickoff_clicked()
{
    coach2robot_info_->MatchType=OUR_KICKOFF;
    coach2robot_info_->MatchMode=OUR_KICKOFF;
    ui->currentState->setText("OUR KICKOFF");
}

void Dialog::on_kickoff_opp_clicked()
{
    coach2robot_info_->MatchType=OPP_KICKOFF;
    coach2robot_info_->MatchMode=OPP_KICKOFF;
    ui->currentState->setText("OPP KICKOFF");
}

void Dialog::on_penalty_clicked()
{
    coach2robot_info_->MatchType=OUR_PENALTY;
    coach2robot_info_->MatchMode=OUR_PENALTY;
    ui->currentState->setText("OUR PENALTY");
}

void Dialog::on_penalty_opp_clicked()
{
    coach2robot_info_->MatchType=OPP_PENALTY;
    coach2robot_info_->MatchMode=OPP_PENALTY;
    ui->currentState->setText("OPP PENALTY");
}

void Dialog::on_corner_clicked()
{
    coach2robot_info_->MatchType=OUR_CORNERKICK;
    coach2robot_info_->MatchMode=OUR_CORNERKICK;
    ui->currentState->setText("OUR CORNER");
}

void Dialog::on_corner_opp_clicked()
{
    coach2robot_info_->MatchType=OPP_CORNERKICK;
    coach2robot_info_->MatchMode=OPP_CORNERKICK;
    ui->currentState->setText("OPP CORNER");
}

void Dialog::on_throwin_clicked()
{
    coach2robot_info_->MatchType=OUR_THROWIN;
    coach2robot_info_->MatchMode=OUR_THROWIN;
    ui->currentState->setText("OUR THROWIN");
}

void Dialog::on_throwin_opp_clicked()
{
    coach2robot_info_->MatchType=OPP_THROWIN;
    coach2robot_info_->MatchMode=OPP_THROWIN;
    ui->currentState->setText("OPP THROWIN");
}

void Dialog::on_freekick_clicked()
{
    coach2robot_info_->MatchType=OUR_FREEKICK;
    coach2robot_info_->MatchMode=OUR_FREEKICK;
    ui->currentState->setText("OUR FREEKICK");
}

void Dialog::on_freekick_opp_clicked()
{
    coach2robot_info_->MatchType=OPP_FREEKICK;
    coach2robot_info_->MatchMode=OPP_FREEKICK;
    ui->currentState->setText("OPP FREEKICK");
}

void Dialog::on_goalkick_clicked()
{
    coach2robot_info_->MatchType=OUR_GOALKICK;
    coach2robot_info_->MatchMode=OUR_GOALKICK;
    ui->currentState->setText("OUR GOALKICK");
}

void Dialog::on_goalkick_opp_clicked()
{
    coach2robot_info_->MatchType=OPP_GOALKICK;
    coach2robot_info_->MatchMode=OPP_GOALKICK;
    ui->currentState->setText("OPP GOALKICK");
}

void Dialog::on_dropball_clicked()
{
    coach2robot_info_->MatchType=DROPBALL;
    coach2robot_info_->MatchMode=DROPBALL;
    ui->currentState->setText("DROPBALL");
}

void Dialog::on_park_clicked()
{
    coach2robot_info_->MatchMode=PARKINGROBOT;
    ui->currentState->setText("PARK ROBOT");
}

void Dialog::on_test_mode_clicked()
{
    coach2robot_info_->MatchMode=TEST;
    ui->currentState->setText("TEST");
    ui->teststate_dis->setText("Test Begin");

    //初始化
    coach2robot_info_->id_A=1;
    coach2robot_info_->id_B=1;
    coach2robot_info_->pointA=nubot::DPoint(0,0);
    coach2robot_info_->pointB=nubot::DPoint(0,0);
    coach2robot_info_->angleA=0;
    coach2robot_info_->angleB=0;
    coach2robot_info_->kick_force=0;
}

void Dialog::on_test_stop_clicked()
{
    coach2robot_info_->TestMode=Test_Stop;
    ui->teststate_dis->setText("Test Stop");

    //置位
//    coach2robot_info_->id_A=1;
//    coach2robot_info_->id_B=1;
//    coach2robot_info_->pointA=nubot::DPoint(0,0);
//    coach2robot_info_->pointB=nubot::DPoint(0,0);
//    coach2robot_info_->angleA=0;
//    coach2robot_info_->angleB=0;
//    coach2robot_info_->kick_force=0;
}

void Dialog::on_location_test_clicked()
{
    if(coach2robot_info_->MatchMode!=TEST)
    {
        notice_->information(this,"Notice","Click The TestMode at first",QMessageBox::Ok,QMessageBox::Ok);
        return;
    }
    coach2robot_info_->TestMode=Location_test;
    ui->teststate_dis->setText("Location Test");
}

void Dialog::on_circle_test_clicked()
{
    if(coach2robot_info_->MatchMode!=TEST)
    {
        notice_->information(this,"Notice","Click The TestMode at first",QMessageBox::Ok,QMessageBox::Ok);
        return;
    }
    coach2robot_info_->TestMode=Circle_test;
    ui->teststate_dis->setText("Circle Test");

    coach2robot_info_->angleA=ui->circle_vel->text().toShort();
    coach2robot_info_->angleB=ui->circle_radius->text().toShort();
}

void Dialog::on_move_mode_clicked()
{
    if(coach2robot_info_->MatchMode!=TEST)
    {
        notice_->information(this,"Notice","Click The TestMode at first",QMessageBox::Ok,QMessageBox::Ok);
        return;
    }
    bool _isDribble=ui->isdribble->checkState();
    bool _isAvoid=ui->isavoidobs->checkState();
    if(!_isDribble && !_isAvoid)
    {
        coach2robot_info_->TestMode=Move_NoBall_NoAvoid;
        ui->teststate_dis->setText("Move NoBall NoAvoid");
    }
    else if(!_isDribble && _isAvoid)
    {
        coach2robot_info_->TestMode=Move_NoBall_Avoid;
        ui->teststate_dis->setText("Move NoBall Avoid");
    }
    else if(_isDribble && !_isAvoid)
    {
        coach2robot_info_->TestMode=Move_Ball_NoAvoid;
        ui->teststate_dis->setText("Move Ball NoAvoid");
    }
    else if(_isDribble && _isAvoid)
    {
        coach2robot_info_->TestMode=Move_Ball_Avoid;
        ui->teststate_dis->setText("Move Ball Avoid");
    }

    coach2robot_info_->id_A=ui->agentA_ID->text().data()->toLatin1()-48;
    coach2robot_info_->pointA.x_=ui->pointAin_X->text().toShort();
    coach2robot_info_->pointA.y_=ui->pointAin_Y->text().toShort();
    coach2robot_info_->pointB.x_=ui->pointBin_X->text().toShort();
    coach2robot_info_->pointB.y_=ui->pointBin_Y->text().toShort();
    coach2robot_info_->angleA=ui->angleAin->text().toShort();
    coach2robot_info_->angleB=ui->angleBin->text().toShort();
}

void Dialog::on_pass_mode_clicked()
{
    if(coach2robot_info_->MatchMode!=TEST)
    {
        notice_->information(this,"Notice","Click The TestMode at first",QMessageBox::Ok,QMessageBox::Ok);
        return;
    }
    coach2robot_info_->TestMode=Pass_Ball;
    ui->teststate_dis->setText("Pass Mode");

    coach2robot_info_->id_A=ui->agentA_ID->text().data()->toLatin1()-48;
    coach2robot_info_->id_B=ui->agentB_ID->text().data()->toLatin1()-48;
    coach2robot_info_->pointA.x_=ui->pointAin_X->text().toShort();
    coach2robot_info_->pointA.y_=ui->pointAin_Y->text().toShort();
    coach2robot_info_->pointB.x_=ui->pointBin_X->text().toShort();
    coach2robot_info_->pointB.y_=ui->pointBin_Y->text().toShort();
}

void Dialog::on_catch_mode_clicked()
{
    if(coach2robot_info_->MatchMode!=TEST)
    {
        notice_->information(this,"Notice","Click The TestMode at first",QMessageBox::Ok,QMessageBox::Ok);
        return;
    }
    coach2robot_info_->TestMode=Catch_Ball;
    ui->teststate_dis->setText("Catch Ball");
    coach2robot_info_->id_A=ui->agentA_ID->text().data()->toLatin1()-48;
}

void Dialog::on_shoot_mode_clicked()
{
    if(coach2robot_info_->MatchMode!=TEST)
    {
        notice_->information(this,"Notice","Click The TestMode at first",QMessageBox::Ok,QMessageBox::Ok);
        return;
    }
    coach2robot_info_->TestMode=Shoot_Ball;
    ui->teststate_dis->setText("Shoot Mode");
    coach2robot_info_->id_A=ui->agentA_ID->text().data()->toLatin1()-48;
    coach2robot_info_->pointB.x_=ui->target_x->text().toShort();
    coach2robot_info_->pointB.y_=ui->target_y->text().toShort();
    coach2robot_info_->kick_force=ui->shoot_force->text().data()->toLatin1()-48;
}

//障碍物显示控制
void Dialog::on_obstacles_clicked()
{
    if(!isObs_display_)
        isObs_display_=true;
    else if(isObs_display_)
    {
        isObs_display_=false;
        for(int i=0;i<OUR_TEAM;i++)
            for(int j=0;j<MAX_OBSNUMBER_CONST;j++)
                obstacle_[i][j]->setPos(900,900);
    }
}

void Dialog::on_pass2catch_clicked()
{
    if(!isP_C_display_)
        isP_C_display_=true;
    else if(isP_C_display_)
    {
        isP_C_display_=false;
        pass2catch_->setLine(900,901,900,901);
    }
}

//场上机器人信息显示
void Dialog::on_Num5_more_clicked()
{
    if(ui->Number5_more->isHidden())
    {
        ui->Number5_more->show();
        ui->Num5_more->setText("main");
    }
    else
    {
        ui->Number5_more->hide();
        ui->Num5_more->setText("more");
    }
}
void Dialog::on_Num4_more_clicked()
{
    if(ui->Number4_more->isHidden())
    {
        ui->Number4_more->show();
        ui->Num4_more->setText("main");
    }
    else
    {
        ui->Number4_more->hide();
        ui->Num4_more->setText("more");
    }
}
void Dialog::on_Num3_more_clicked()
{
    if(ui->Number3_more->isHidden())
    {
        ui->Number3_more->show();
        ui->Num3_more->setText("main");
    }
    else
    {
        ui->Number3_more->hide();
        ui->Num3_more->setText("more");
    }
}
void Dialog::on_Num2_more_clicked()
{
    if(ui->Number2_more->isHidden())
    {
        ui->Number2_more->show();
        ui->Num2_more->setText("main");
    }
    else
    {
        ui->Number2_more->hide();
        ui->Num2_more->setText("more");
    }
}
void Dialog::on_Num1_more_clicked()
{
    if(ui->Number1_more->isHidden())
    {
        ui->Number1_more->show();
        ui->Num1_more->setText("main");
    }
    else
    {
        ui->Number1_more->hide();
        ui->Num1_more->setText("more");
    }
}

//队伍选择
void Dialog::on_cyan_clicked()
{
    teamflag_=1;
    ui->cyan->setStyleSheet("color:rgb(255,255,255);background-color:rgb(11, 246, 230);border:0;border-radius:10px;");
    ui->magenta->setStyleSheet("color:rgb(255,255,255);border:0;border-radius:10px;");
}

void Dialog::on_magenta_clicked()
{
    teamflag_=0;
    ui->magenta->setStyleSheet("color:rgb(255,255,255);background-color:rgb(245, 12, 198);border:0;border-radius:10px;");
    ui->cyan->setStyleSheet("color:rgb(255,255,255);border:0;border-radius:10px;");
}

//连接裁判盒
void Dialog::on_connectRefe_clicked()
{
    if(!isConnect_RefBox_)
    {
        QString IP=ui->IP_in->text();
        qDebug()<<IP;
        quint16 prot=28097;
        tcpSocket_->abort();
        tcpSocket_->connectToHost(IP,prot);

        connect(tcpSocket_, SIGNAL(readyRead()), this, SLOT(OnReceive_()));    //tcp/ip通信时用到的槽函数
        ui->connectRefe->setText("Disconnect");
        isConnect_RefBox_=true;

        coach2robot_info_->MatchMode=STOPROBOT;                    //连接裁判盒时，置位比赛模式，防止机器人乱跑
        coach2robot_info_->MatchType=STOPROBOT;
        ui->currentState->setText("STOP ROBOT");

        ui->IP_in->clearFocus();
        qDebug()<<"RefBox_Connected";
        disableButton_();
    }
    else
    {
        tcpSocket_->disconnectFromHost();
        disconnect(tcpSocket_, SIGNAL(readyRead()), this, SLOT(OnReceive_()));
        ui->connectRefe->setText("Connect");                //断开链接的同时停止上传
        ui->upload->setText("Upload");
        isConnect_RefBox_=false;
        isUpload_worldmodel_=false;
        qDebug()<<"RefBox_Disconnected";
        qDebug()<<"Stop_upload";
        enableButton_();
    }
}

void Dialog::on_upload_clicked()
{
    if(!isUpload_worldmodel_&&isConnect_RefBox_)
    {
        ui->upload->setText("StopUp");
        isUpload_worldmodel_=true;
        qDebug()<<"Upload_worldmodel";
    }
    else
    {
        if(!isConnect_RefBox_)
            notice_->information(this,"Notice","Connect RefBox at first",QMessageBox::Ok,QMessageBox::Ok);
        ui->upload->setText("Upload");
        isUpload_worldmodel_=false;
        qDebug()<<"Stop_upload";
    }
}

void Dialog::on_change_ground_clicked()
{
    if(groundflag_==1)
    {
        ui->change_ground->setStyleSheet("border-image: url(:/right2left.png)");
        groundflag_=-1;
    }
    else if (groundflag_==-1)
    {
        ui->change_ground->setStyleSheet("border-image: url(:/left2right.png)");
        groundflag_=1;
    }
}

void Dialog::on_field_home_clicked()
{
    if(!isAtHome_)
    {
        field_->setPixmap(field_img_home_);     //改为奥拓楼球场
        isAtHome_=true;
    }
    else if (isAtHome_)
    {
        field_->setPixmap(field_img_init_);
        isAtHome_=false;
    }
}

//用于显示所有机器人和球的位置和速度信息
void Dialog::showAll_info_()
{
    static bool last_valid[OUR_TEAM];
    static bool last_dribble[OUR_TEAM];
    QString current_actions;
    if(robot2coach_info_->Catch_id_>0 && robot2coach_info_->Catch_id_<OUR_TEAM)
        ui->Catch_num_dis->setText(QString("%1").arg(robot2coach_info_->Catch_id_));
    else
        ui->Catch_num_dis->setText("NA");
    if(robot2coach_info_->Pass_id_>0 && robot2coach_info_->Pass_id_<OUR_TEAM)
        ui->Pass_num_dis->setText(QString("%1").arg(robot2coach_info_->Pass_id_));
    else
        ui->Pass_num_dis->setText("NA");


    for(int i=0;i<OUR_TEAM;i++)
    {
        switch (robot2coach_info_->RobotInfo_[i].getCurrentAction())
        {
        case 0: current_actions="Stucked";break;
        case 1: current_actions="Penalty";break;
        case 2: current_actions="CanNotSeeBall";break;
        case 3: current_actions="SeeNotDribbleBall";break;
        case 4: current_actions="TurnForShoot";break;
        case 5: current_actions="TurnForShoot_Robot";break;
        case 6: current_actions="AtShootSituation";break;
        case 7: current_actions="TurnToPass";break;
        case 8: current_actions="TurnToPass_Robot";break;
        case 9: current_actions="StaticPass";break;
        case 10: current_actions="AvoidObs";break;
        case 11: current_actions="Catch_Positioned";break;
        case 12:
            if(i==0)             //专门针对守门员的显示
                current_actions="ReToOrigin";
            else
                current_actions="Positioned";
            break;
        case 13: current_actions="Positioned_Static";break;
        case 14: current_actions="KickCoop";break;
        case 15: current_actions="KickCoop_turn";break;
        case 16:
            if(i==0)
                current_actions="BlockBall";
            else
                current_actions="CatchBall";
            break;
        case 17: current_actions="CatchBall_slow";break;
        case 18: current_actions="CircleTest";break;
        case 19: current_actions="MoveWithBall";break;
        case 20: current_actions="TeleopJoy";break;
        case 21: current_actions="No_Action";break;
        default: break;
        }

        infoShow_[i]=current_actions;
    }

    for(int i=0;i<OUR_TEAM;i++)
    {
        if(last_valid[i])
        {
            short distance=_robot_pos[i].distance(_ball_pos[i]);
            R_POS_Browser[i]->setText(QString("%1  %2").arg(_robot_pos[i].x_,4).arg(_robot_pos[i].y_,4));
            R_ORI_Browser[i]->setText(QString("%1").arg(_robot_ori[i],4));
            B_POS_Browser[i]->setText(QString("%1  %2").arg(_ball_pos[i].x_,4).arg(_ball_pos[i].y_,4));
            Action_Browser[i]->setText(infoShow_[i]);
            R_VEL_Browser[i]->setText(QString("%1  %2").arg(_robot_vel[i].x_,4).arg(_robot_vel[i].y_,4));
            B_VEL_Browser[i]->setText(QString("%1  %2").arg(_ball_vel[i].x_,4).arg(_ball_vel[i].y_,4));
            Distance_Browser[i]->setText(QString("%1").arg(distance,3));
        }

        if(last_valid[i]!=robot2coach_info_->RobotInfo_[i].isValid())
        {
            if(robot2coach_info_->RobotInfo_[i].isValid())
                Group_Label[i]->setStyleSheet("QGroupBox {color:rgb(255,255,255)}");
            else
                Group_Label[i]->setStyleSheet("QGroupBox {color:rgb(51,51,51)}");
        }
        else if(robot2coach_info_->RobotInfo_[i].isValid()&&(last_dribble[i]!=robot2coach_info_->RobotInfo_[i].getDribbleState()))
        {
            if(robot2coach_info_->RobotInfo_[i].getDribbleState())
                Group_Label[i]->setStyleSheet("QGroupBox {color:rgb(233,34,34)}");
            else
                Group_Label[i]->setStyleSheet("QGroupBox {color:rgb(255,255,255)}");
        }

        last_valid[i]=robot2coach_info_->RobotInfo_[i].isValid();
        last_dribble[i]=robot2coach_info_->RobotInfo_[i].getDribbleState();
    }
}

//在多个机器人都看到球的情况下做融合，仅用于绘图
int Dialog::ballPos_fuse()
{
    QVector<nubot::DPoint> ball_pos;
    QVector<nubot::DPoint> robot_pos;
    QVector<int> robot_num;
    float robot2ball_dis=20000;                        //机器人都看到球的最短距离
    int num=0;

    for(int i=0;i<OUR_TEAM;i++)                                                   //如果机器人看到了球就将其压到堆栈中
        if(robot2coach_info_->BallInfo_[i].isLocationKnown()&&robot2coach_info_->RobotInfo_[i].isValid())
        {
            ball_pos.push_back(robot2coach_info_->BallInfo_[i].getGlobalLocation());
            robot_pos.push_back(robot2coach_info_->RobotInfo_[i].getLocation());
            robot_num.push_back(i);
        }
    if(ball_pos.size())
    {
        for(int i=0;i<ball_pos.size();i++)
            if(ball_pos[i].distance(robot_pos[i])<robot2ball_dis)
            {
                robot2ball_dis=ball_pos[i].distance(robot_pos[i]);
                num=robot_num[i];
            }

        return (num+1);
    }
    else
        return num;

}

//根据xml解析结果发布coach指令
void Dialog::RefBox_Info()
{
    char chafinal=json_parse_->chafinal_;
   // qDebug()<<chafinal;
   // score_cyan_=json_parse_->score_cyan_;
   // score_magenta_=json_parse_->score_magenta_;
    switch (chafinal)
    {
    case COMM_START:
       // coach2robot_info_->MatchType=coach2robot_info_->MatchMode;
        coach2robot_info_->MatchMode=STARTROBOT;
        ui->currentState->setText("START ROBOT");
        break;
    case COMM_FIRST_HALF:
      //  coach2robot_info_->MatchType=coach2robot_info_->MatchMode;
        coach2robot_info_->MatchMode=STARTROBOT;
        ui->currentState->setText("FIRST HALF");
        break;
    case COMM_SECOND_HALF:
      //  coach2robot_info_->MatchType=coach2robot_info_->MatchMode;
        coach2robot_info_->MatchMode=STARTROBOT;
        ui->currentState->setText("SECOND HALF");
        break;
     case COMM_STOP:
        coach2robot_info_->MatchMode=STOPROBOT;
        ui->currentState->setText("STOP ROBOT");
        break;
     case COMM_HALT:
        coach2robot_info_->MatchMode=STOPROBOT;
        ui->currentState->setText("Halt");
        break;
     case COMM_READY:
        ui->currentState->setText("READY");
        break;
     case COMM_DROPPED_BALL:
        coach2robot_info_->MatchType=DROPBALL;
        coach2robot_info_->MatchMode=DROPBALL;
        ui->currentState->setText("DROPBALL");
        break;
     case COMM_GOAL_MAGENTA:
        score_magenta_++;
        //coach2robot_info_->MatchMode=STOPROBOT;
        ui->Score0->display(score_magenta_);
        break;
     case COMM_GOAL_CYAN:
        score_cyan_++;
        //coach2robot_info_->MatchMode=STOPROBOT;
        ui->Score1->display(score_cyan_);
        break;
     case COMM_SUBGOAL_MAGENTA:
        score_magenta_--;
        //coach2robot_info_->MatchMode=STOPROBOT;
        ui->Score0->display(score_magenta_);
        break;
     case COMM_SUBGOAL_CYAN:
        score_cyan_--;
        //coach2robot_info_->MatchMode=STOPROBOT;
        ui->Score1->display(score_cyan_);
        break;
     case COMM_KICKOFF_MAGENTA:
        if(teamflag_)
        {
            coach2robot_info_->MatchType=OPP_KICKOFF;
            coach2robot_info_->MatchMode=OPP_KICKOFF;
            ui->currentState->setText("OPP KICKOFF");
        }
        else
        {
            coach2robot_info_->MatchType=OUR_KICKOFF;
            coach2robot_info_->MatchMode=OUR_KICKOFF;
            ui->currentState->setText("OUR KICKOFF");
        }
        break;
     case COMM_KICKOFF_CYAN:
        if(teamflag_)
        {
            coach2robot_info_->MatchType=OUR_KICKOFF;
            coach2robot_info_->MatchMode=OUR_KICKOFF;
            ui->currentState->setText("OUR KICKOFF");
        }
        else
        {
            coach2robot_info_->MatchType=OPP_KICKOFF;
            coach2robot_info_->MatchMode=OPP_KICKOFF;
            ui->currentState->setText("OPP KICKOFF");
        }
        break;
     case COMM_FREEKICK_MAGENTA:
        if(teamflag_)
        {
            coach2robot_info_->MatchType=OPP_FREEKICK;
            coach2robot_info_->MatchMode=OPP_FREEKICK;
            ui->currentState->setText("OPP FREEKICK");
        }
        else
        {
            coach2robot_info_->MatchType=OUR_FREEKICK;
            coach2robot_info_->MatchMode=OUR_FREEKICK;
            ui->currentState->setText("OUR FREEKICK");
        }
        break;
    case COMM_FREEKICK_CYAN:
        if(teamflag_)
        {
            coach2robot_info_->MatchType=OUR_FREEKICK;
            coach2robot_info_->MatchMode=OUR_FREEKICK;
            ui->currentState->setText("OUR FREEKICK");
        }
        else
        {
            coach2robot_info_->MatchType=OPP_FREEKICK;
            coach2robot_info_->MatchMode=OPP_FREEKICK;
            ui->currentState->setText("OPP FREEKICK");
        }
        break;
    case COMM_GOALKICK_MAGENTA:
        if(teamflag_)
        {
            coach2robot_info_->MatchType = OPP_GOALKICK;
            coach2robot_info_->MatchMode = OPP_GOALKICK;
            ui->currentState->setText("OPP GOALKICK");
        }
        else
        {
            coach2robot_info_->MatchType=OUR_GOALKICK;
            coach2robot_info_->MatchMode=OUR_GOALKICK;
            ui->currentState->setText("OUR GOALKICK");
        }
        break;
    case COMM_GOALKICK_CYAN:
        if(teamflag_)
        {
            coach2robot_info_->MatchType=OUR_GOALKICK;
            coach2robot_info_->MatchMode=OUR_GOALKICK;
            ui->currentState->setText("OUR GOALKICK");
        }
        else
        {
            coach2robot_info_->MatchType=OPP_GOALKICK;
            coach2robot_info_->MatchMode=OPP_GOALKICK;
            ui->currentState->setText("OPP GOALKICK");
        }
        break;
    case COMM_THROWIN_MAGENTA:
        if(teamflag_)
        {
            coach2robot_info_->MatchType=OPP_THROWIN;
            coach2robot_info_->MatchMode=OPP_THROWIN;
            ui->currentState->setText("OPP THROWIN");
        }
        else
        {
            coach2robot_info_->MatchType=OUR_THROWIN;
            coach2robot_info_->MatchMode=OUR_THROWIN;
            ui->currentState->setText("OUR THROWIN");
        }
        break;
    case COMM_THROWIN_CYAN:
        if(teamflag_)
        {
            coach2robot_info_->MatchType=OUR_THROWIN;
            coach2robot_info_->MatchMode=OUR_THROWIN;
            ui->currentState->setText("OUR THROWIN");
        }
        else
        {
            coach2robot_info_->MatchType=OPP_THROWIN;
            coach2robot_info_->MatchMode=OPP_THROWIN;
            ui->currentState->setText("OPP THROWIN");
        }
        break;
    case COMM_CORNER_MAGENTA:
        if(teamflag_)
        {
            coach2robot_info_->MatchType=OPP_CORNERKICK;
            coach2robot_info_->MatchMode=OPP_CORNERKICK;
            ui->currentState->setText("OPP CORNER");
        }
        else
        {
            coach2robot_info_->MatchType=OUR_CORNERKICK;
            coach2robot_info_->MatchMode=OUR_CORNERKICK;
            ui->currentState->setText("OUR CORNER");
        }
        break;
    case COMM_CORNER_CYAN:
        if(teamflag_)
        {
            coach2robot_info_->MatchType=OUR_CORNERKICK;
            coach2robot_info_->MatchMode=OUR_CORNERKICK;
            ui->currentState->setText("OUR CORNER");
        }
        else
        {
            coach2robot_info_->MatchType=OPP_CORNERKICK;
            coach2robot_info_->MatchMode=OPP_CORNERKICK;
            ui->currentState->setText("OPP CORNER");
        }
        break;
    case COMM_PENALTY_MAGENTA:
        if(teamflag_)
        {
            coach2robot_info_->MatchType=OPP_PENALTY;
            coach2robot_info_->MatchMode=OPP_PENALTY;
            ui->currentState->setText("OPP PENALTY");
        }
        else
        {
            coach2robot_info_->MatchType=OUR_PENALTY;
            coach2robot_info_->MatchMode=OUR_PENALTY;
            ui->currentState->setText("OUR PENALTY");
        }
        break;
    case COMM_PENALTY_CYAN:
        if(teamflag_)
        {
            coach2robot_info_->MatchType=OUR_PENALTY;
            coach2robot_info_->MatchMode=OUR_PENALTY;
            ui->currentState->setText("OUR PENALTY");
        }
        else
        {
            coach2robot_info_->MatchType=OPP_PENALTY;
            coach2robot_info_->MatchMode=OPP_PENALTY;
            ui->currentState->setText("OPP PENALTY");
        }
        break;
    case COMM_CANCEL:
        ui->currentState->setText("CANCEL");
        break;
    case COMM_PARKING:
        coach2robot_info_->MatchMode=PARKINGROBOT;
        ui->currentState->setText("PARKING");
        break;
    case COMM_HALF_TIME:
        coach2robot_info_->MatchMode=STOPROBOT;
        ui->currentState->setText("END_PART");
        break;
    case COMM_END_GAME:
        ui->currentState->setText("GOOD_GAME");
        break;
    default:
        break;
    }
   // ui->Score0->display(score_magenta_);
   // ui->Score1->display(score_cyan_);
}

//连接裁判盒后，coach不能主动发布命令
void Dialog::disableButton_()
{
    ui->kickoff_opp->setEnabled(false);
    ui->penalty_opp->setEnabled(false);
    ui->corner_opp->setEnabled(false);
    ui->throwin_opp->setEnabled(false);
    ui->freekick_opp->setEnabled(false);
    ui->goalkick_opp->setEnabled(false);
    ui->park->setEnabled(false);
    ui->kickoff->setEnabled(false);
    ui->penalty->setEnabled(false);
    ui->corner->setEnabled(false);
    ui->throwin->setEnabled(false);
    ui->freekick->setEnabled(false);
    ui->goalkick->setEnabled(false);
    ui->dropball->setEnabled(false);
    ui->startButton->setEnabled(false);
    ui->stopButton->setEnabled(false);
    ui->test_mode->setEnabled(false);
}

void Dialog::enableButton_()
{
    ui->kickoff_opp->setEnabled(true);
    ui->penalty_opp->setEnabled(true);
    ui->corner_opp->setEnabled(true);
    ui->throwin_opp->setEnabled(true);
    ui->freekick_opp->setEnabled(true);
    ui->goalkick_opp->setEnabled(true);
    ui->park->setEnabled(true);
    ui->kickoff->setEnabled(true);
    ui->penalty->setEnabled(true);
    ui->corner->setEnabled(true);
    ui->throwin->setEnabled(true);
    ui->freekick->setEnabled(true);
    ui->goalkick->setEnabled(true);
    ui->dropball->setEnabled(true);
    ui->startButton->setEnabled(true);
    ui->stopButton->setEnabled(true);
    ui->test_mode->setEnabled(true);
}

//接收jason的响应函数
void Dialog::OnReceive_()
{
    QTcpSocket *socket = static_cast<QTcpSocket*>(sender());
    QByteArray *buffer = buffers.value(socket);

    while (socket->bytesAvailable() > 0)
    {
        buffer->clear();
        buffer->append(socket->readAll());
        for(int i = 0 ; i < buffer->size() ;i++)
        {
            json_parse_->chafinal_ = buffer->data()[i];
            RefBox_Info();
            ui->text_disply->setStyleSheet("color:rgb(255,0,0);");
            qDebug()<< json_parse_->chafinal_<<buffer->size();
        }
     /*   if(1)//buffer->contains('\0'))
        {
            qint32 size = buffer->indexOf('\0');
            QByteArray data = buffer->mid(0, size);
            qDebug()<<data;

            buffer->remove(0, size + 1);
            if(json_parse_->parseJSON_(data))
            {
                RefBox_Info();
                ui->text_disply->setStyleSheet("color:rgb(255,0,0);");
                qDebug()<<data;
            }
        }*/
    }
}

//返回链接错误
void Dialog::displayError_(QAbstractSocket::SocketError)
{
    QString error = tcpSocket_->errorString();
    notice_->information(this,"Notice",error,QMessageBox::Ok,QMessageBox::Ok);
    tcpSocket_->close();

    ui->connectRefe->setText("Connect");                //断开链接的同时停止上传
    ui->upload->setText("Upload");
    isConnect_RefBox_=false;
    isUpload_worldmodel_=false;


    qDebug()<<"RefBox_Disconnected";
    qDebug()<<"Stop_upload";
    enableButton_();
}

//延时函数，目前没有用
void Dialog::buttonDelay_()              //每隔450ms按钮复位
{
    static int count=0;
    if(count==100)
    {
        ui->startButton->setDown(false);
        count==0;
    }
    else
        count++;
}

void Dialog::restItems_()
{
    if(current_valid_robots==0)
        ball_->setPos(900,900);                             //初始位置放到(900,900),不出现在视野里
        velocity_->setLine(900,901,900,901);
    if(current_valid_robots==0||!robot2coach_info_->isPass_valid_)
        pass2catch_->setLine(900,901,900,901);
    for(int i=0;i<OUR_TEAM;i++)
        if(!robot2coach_info_->RobotInfo_[i].isValid()||display_choice_!=0)
        {
            robot_[i]->setPos(900,900);
            role_[i]->setPos(900,900);
            for(int j=0;j<MAX_OBSNUMBER_CONST;j++)
                obstacle_[i][j]->setPos(900,900);
        }
}

void Dialog::init_style_()
{
    ui->tab_change->setStyleSheet("QTabWidget:pane {border-top:0px solid #e8f3f9;background:transparent;}"               //控制界面透明
                                 "QTabBar::tab:pane {border-top:0px solid #e8f3f9;background:transparent;}");
    ui->isdribble->setStyleSheet("QCheckBox{color:rgb(255,255,255); background:transparent}");
    ui->isavoidobs->setStyleSheet("QCheckBox{color:rgb(255,255,255); background:transparent}");

    QPixmap icon_pix_control,icon_pix_test,icon_pix_vs;                                                                  //tab图标
    QIcon icon_control,icon_test,icon_vs;

    icon_pix_control.load(":/control.png");
    icon_control.addPixmap(icon_pix_control);

    icon_pix_test.load(":/test.png");
    icon_test.addPixmap(icon_pix_test);

    icon_pix_vs.load(":/vs.png");
    icon_vs.addPixmap(icon_pix_vs);

    ui->tab_change->tabBar()->setTabIcon(0,icon_control);
    ui->tab_change->tabBar()->setTabIcon(1,icon_test);
    ui->tab_change->tabBar()->setTabIcon(2,icon_vs);
    ui->tab_change->tabBar()->setIconSize(QSize(82,35));

    ui->Number5_more->setStyleSheet("background-image: url(:/num5.png)");
    ui->Number4_more->setStyleSheet("background-image: url(:/num4.png)");
    ui->Number3_more->setStyleSheet("background-image: url(:/num3.png)");
    ui->Number2_more->setStyleSheet("background-image: url(:/num2.png)");
    ui->Number1_more->setStyleSheet("background-image: url(:/num1.png)");

    ui->Number5_more->hide();
    ui->Number4_more->hide();
    ui->Number3_more->hide();
    ui->Number2_more->hide();
    ui->Number1_more->hide();

    R_POS_Browser<<ui->Num1_R_POS_Browser<<ui->Num2_R_POS_Browser<<ui->Num3_R_POS_Browser<<ui->Num4_R_POS_Browser<<ui->Num5_R_POS_Browser;
    R_ORI_Browser<<ui->Num1_R_ORI_Browser<<ui->Num2_R_ORI_Browser<<ui->Num3_R_ORI_Browser<<ui->Num4_R_ORI_Browser<<ui->Num5_R_ORI_Browser;
    B_POS_Browser<<ui->Num1_B_POS_Browser<<ui->Num2_B_POS_Browser<<ui->Num3_B_POS_Browser<<ui->Num4_B_POS_Browser<<ui->Num5_B_POS_Browser;
    Action_Browser<<ui->Num1_Action_Browser<<ui->Num2_Action_Browser<<ui->Num3_Action_Browser<<ui->Num4_Action_Browser<<ui->Num5_Action_Browser;

    R_VEL_Browser<<ui->Num1_R_VEL_Browser<<ui->Num2_R_VEL_Browser<<ui->Num3_R_VEL_Browser<<ui->Num4_R_VEL_Browser<<ui->Num5_R_VEL_Browser;
    B_VEL_Browser<<ui->Num1_B_VEL_Browser<<ui->Num2_B_VEL_Browser<<ui->Num3_B_VEL_Browser<<ui->Num4_B_VEL_Browser<<ui->Num5_B_VEL_Browser;
    Distance_Browser<<ui->disNum1_Browser<<ui->disNum2_Browser<<ui->disNum3_Browser<<ui->disNum4_Browser<<ui->disNum5_Browser;

    R_POS_Label<<ui->Num1_R_POS<<ui->Num2_R_POS<<ui->Num3_R_POS<<ui->Num4_R_POS<<ui->Num5_R_POS;
    R_ORI_Label<<ui->Num1_R_ORI<<ui->Num2_R_ORI<<ui->Num3_R_ORI<<ui->Num4_R_ORI<<ui->Num5_R_ORI;
    B_POS_Label<<ui->Num1_B_POS<<ui->Num2_B_POS<<ui->Num3_B_POS<<ui->Num4_B_POS<<ui->Num5_B_POS;
    Action_Label<<ui->Num1_Action<<ui->Num2_Action<<ui->Num3_Action<<ui->Num4_Action<<ui->Num5_Action;

    R_VEL_Label<<ui->Num1_R_VEL<<ui->Num2_R_VEL<<ui->Num3_R_VEL<<ui->Num4_R_VEL<<ui->Num5_R_VEL;
    B_VEL_Label<<ui->Num1_B_VEL<<ui->Num2_B_VEL<<ui->Num3_B_VEL<<ui->Num4_B_VEL<<ui->Num5_B_VEL;
    Distance_Label<<ui->disNum1<<ui->disNum2<<ui->disNum3<<ui->disNum4<<ui->disNum5;
    Group_Label<<ui->Number1<<ui->Number2<<ui->Number3<<ui->Number4<<ui->Number5;

    for(int i=0;i<OUR_TEAM;i++)
    {
        R_POS_Browser[i]->setStyleSheet("QTextBrowser {color:rgb(255,255,255); border:0; background:transparent}");
        R_POS_Label[i]->setStyleSheet("QLabel {color:rgb(255,255,255)}");

        R_ORI_Browser[i]->setStyleSheet("QTextBrowser {color:rgb(255,255,255); border:0; background:transparent}");
        R_ORI_Label[i]->setStyleSheet("QLabel {color:rgb(255,255,255)}");

        B_POS_Browser[i]->setStyleSheet("QTextBrowser {color:rgb(255,255,255); border:0; background:transparent}");
        B_POS_Label[i]->setStyleSheet("QLabel {color:rgb(255,255,255)}");

        Action_Browser[i]->setStyleSheet("QTextBrowser {color:rgb(255,255,255); border:0; background:transparent}");
        Action_Label[i]->setStyleSheet("QLabel {color:rgb(255,255,255)}");

        R_VEL_Browser[i]->setStyleSheet("QTextBrowser {color:rgb(255,255,255); border:0; background:transparent}");
        R_VEL_Label[i]->setStyleSheet("QLabel {color:rgb(255,255,255)}");

        B_VEL_Browser[i]->setStyleSheet("QTextBrowser {color:rgb(255,255,255); border:0; background:transparent}");
        B_VEL_Label[i]->setStyleSheet("QLabel {color:rgb(255,255,255)}");

        Distance_Browser[i]->setStyleSheet("QTextBrowser {color:rgb(255,255,255); border:0; background:transparent}");
        Distance_Label[i]->setStyleSheet("QLabel {color:rgb(255,255,255)}");

        Group_Label[i]->setStyleSheet("QGroupBox {color:rgb(51,51,51)}");
    }
}
