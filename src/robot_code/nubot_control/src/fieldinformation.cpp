#include "nubot/nubot_control/fieldinformation.h"
using namespace nubot;
const int simLOCATIONERROR = 0;

FieldInformation::FieldInformation()
{
    xline_.push_back(FIELD_XLINE1);  /** 900，0*/   // 与x轴垂直的直线
    xline_.push_back(FIELD_XLINE2);
    xline_.push_back(FIELD_XLINE3);
    xline_.push_back(FIELD_XLINE4);
    xline_.push_back(FIELD_XLINE5);
    xline_.push_back(FIELD_XLINE6);
    xline_.push_back(FIELD_XLINE7); /** -900，6*/

    yline_.push_back(FIELD_YLINE1);  /** 600，0 */  // 与y轴垂直的直线
    yline_.push_back(FIELD_YLINE2);
    yline_.push_back(FIELD_YLINE3);
    yline_.push_back(FIELD_YLINE4);
    yline_.push_back(FIELD_YLINE5);
    yline_.push_back(FIELD_YLINE6); /** -600，5 */

    /** 场地中存在的所有的白线线段*/
    x_white_line_.push_back(LineSegment(DPoint(xline_[0],yline_[5]),DPoint(xline_[0],yline_[0])));
    x_white_line_.push_back(LineSegment(DPoint(xline_[1],yline_[3]),DPoint(xline_[1],yline_[2])));
    x_white_line_.push_back(LineSegment(DPoint(xline_[2],yline_[4]),DPoint(xline_[2],yline_[1])));
    x_white_line_.push_back(LineSegment(DPoint(xline_[3],yline_[5]),DPoint(xline_[3],yline_[0])));
    x_white_line_.push_back(LineSegment(DPoint(xline_[4],yline_[4]),DPoint(xline_[4],yline_[1])));
    x_white_line_.push_back(LineSegment(DPoint(xline_[5],yline_[3]),DPoint(xline_[5],yline_[2])));
    x_white_line_.push_back(LineSegment(DPoint(xline_[6],yline_[5]),DPoint(xline_[6],yline_[0])));

    y_white_line_.push_back(LineSegment(DPoint(xline_[6],yline_[5]),DPoint(xline_[0],yline_[5])));
    y_white_line_.push_back(LineSegment(DPoint(xline_[6],yline_[0]),DPoint(xline_[0],yline_[0])));
    y_white_line_.push_back(LineSegment(DPoint(xline_[6],yline_[4]),DPoint(xline_[4],yline_[4])));
    y_white_line_.push_back(LineSegment(DPoint(xline_[0],yline_[4]),DPoint(xline_[2],yline_[4])));
    y_white_line_.push_back(LineSegment(DPoint(xline_[6],yline_[1]),DPoint(xline_[4],yline_[1])));
    y_white_line_.push_back(LineSegment(DPoint(xline_[0],yline_[1]),DPoint(xline_[2],yline_[1])));
    y_white_line_.push_back(LineSegment(DPoint(xline_[6],yline_[3]),DPoint(xline_[5],yline_[3])));
    y_white_line_.push_back(LineSegment(DPoint(xline_[0],yline_[3]),DPoint(xline_[1],yline_[3])));
    y_white_line_.push_back(LineSegment(DPoint(xline_[6],yline_[2]),DPoint(xline_[5],yline_[2])));
    y_white_line_.push_back(LineSegment(DPoint(xline_[0],yline_[2]),DPoint(xline_[1],yline_[2])));

    centercircle_.radius_=FIELD_CENTER_RADIUS;
    centercircle_.center_=DPoint2d(0,0);

    postcircle_.resize(4);
    for(size_t i=0; i< 4;i++)
        postcircle_[i].radius_=FIELD_POST_RADIUS;
    postcircle_[0].center_=DPoint2d(xline_[0],-yline_[0]);
    postcircle_[1].center_=DPoint2d(xline_[0],yline_[0]);
    postcircle_[2].center_=DPoint2d(-xline_[0],yline_[0]);
    postcircle_[3].center_=DPoint2d(-xline_[0],-yline_[0]);

    /** 守门员处的几个关键位置*/
    oppGoal_[GOAL_UPPER]    = DPoint(xline_[0],120);
    oppGoal_[GOAL_MIDUPPER] = DPoint(xline_[0],60);
    oppGoal_[GOAL_MIDDLE]   = DPoint(xline_[0],0);
    oppGoal_[GOAL_MIDLOWER] = DPoint(xline_[0],-60);
    oppGoal_[GOAL_LOWER]    = DPoint(xline_[0],-120);

    ourGoal_[GOAL_UPPER]    = DPoint(xline_[6],120);
    ourGoal_[GOAL_MIDUPPER] = DPoint(xline_[6],60);
    ourGoal_[GOAL_MIDDLE]   = DPoint(xline_[6],0);
    ourGoal_[GOAL_MIDLOWER] = DPoint(xline_[6],-60);
    ourGoal_[GOAL_LOWER]    = DPoint(xline_[6],-120);


    our_penaltyarea_[0] = DPoint(xline_[6],yline_[1]);  //(-900,325)    我方大禁区
    our_penaltyarea_[1] = DPoint(xline_[4],yline_[1]);  //(-675,325)
    our_penaltyarea_[2] = DPoint(xline_[4],yline_[4]);  //(-675,-325)
    our_penaltyarea_[3] = DPoint(xline_[6],yline_[4]);  //(-900,-325)
    ourPenaltyLine_[0] = LineSegment(our_penaltyarea_[0], our_penaltyarea_[1]); //我方大禁区上横线
    ourPenaltyLine_[1] = LineSegment(our_penaltyarea_[1], our_penaltyarea_[2]); //我方大禁区竖线
    ourPenaltyLine_[2] = LineSegment(our_penaltyarea_[2], our_penaltyarea_[3]); //我方大禁区下横线

    opp_penaltyarea_[0] = DPoint(xline_[0],yline_[1]);  // 对方大禁区
    opp_penaltyarea_[1] = DPoint(xline_[2],yline_[1]);
    opp_penaltyarea_[2] = DPoint(xline_[2],yline_[4]);
    opp_penaltyarea_[3] = DPoint(xline_[0],yline_[4]);
    oppPenaltyLine_[0] = LineSegment(opp_penaltyarea_[0], opp_penaltyarea_[1]); //我方大禁区上横线
    oppPenaltyLine_[1] = LineSegment(opp_penaltyarea_[1], opp_penaltyarea_[2]); //我方大禁区竖线
    oppPenaltyLine_[2] = LineSegment(opp_penaltyarea_[2], opp_penaltyarea_[3]); //我方大禁区下横线

    our_goalarea_[0] = DPoint(xline_[6],yline_[2]);   //(-900,175)   我方小禁区
    our_goalarea_[1] = DPoint(xline_[5],yline_[2]);   //(-825,175)
    our_goalarea_[2] = DPoint(xline_[5],yline_[3]);   //(-825,-175)
    our_goalarea_[3] = DPoint(xline_[6],yline_[3]);   //(-900,-175)

    opp_goalarea_[0] = DPoint(xline_[0],yline_[2]);   //(900,175)    对方小禁区
    opp_goalarea_[1] = DPoint(xline_[1],yline_[2]);   //(825,175)
    opp_goalarea_[2] = DPoint(xline_[1],yline_[3]);   //(825,-175)
    opp_goalarea_[3] = DPoint(xline_[0],yline_[3]);   //(900,-175)

    ourPenaltyPt_ = DPoint(-600,0);
    oppPenaltyPt_ = DPoint(600,0);
}

FieldInformation::FieldInformation(string infopath)
{

}

/* 是否在场地内 */
bool FieldInformation::isInInterField(DPoint world_pt,double expand_len)
{
    return (world_pt.x_>xline_[6]-expand_len &&
            world_pt.x_<xline_[0]+expand_len &&
            world_pt.y_>yline_[5]-expand_len &&
            world_pt.y_<yline_[0]+expand_len);
}

bool FieldInformation::isInInterField2(DPoint world_pt, double expand_len, double expand_width)
{
    return (world_pt.x_>xline_[6]-expand_len &&
            world_pt.x_<xline_[0]+expand_len &&
            world_pt.y_>yline_[5]-expand_width&&
            world_pt.y_<yline_[0]+expand_width);
}

/* 是否在场地外 */
bool FieldInformation::isInOuterField(DPoint world_pt,double expand_len)
{
    return (world_pt.x_<xline_[6]-expand_len ||
            world_pt.x_>xline_[0]+expand_len ||
            world_pt.y_<yline_[5]-expand_len ||
            world_pt.y_>yline_[0]+expand_len);
}

/* 是否在对方半场 */
bool FieldInformation::isOppField(DPoint world_pt)
{
    bool rtvl = false;
    static bool inoppfield = false;

    if(!inoppfield && world_pt.x_ > 0)
    {
        inoppfield  =  true;
    }
    else if(inoppfield && world_pt.x_ >  -simLOCATIONERROR)
    {
        inoppfield =  true;
    }
    else
    {
        inoppfield =  false;
    }
    rtvl = inoppfield;

    return rtvl;
}

/* 是否在己方半场*/
bool FieldInformation::isOurField(DPoint world_pt)
{
    bool rtvl = false;
    static bool inourfield = false;

    if(!inourfield && world_pt.x_ < 0)
    {
        inourfield  =  true;
    }
    else if(inourfield && world_pt.x_ < simLOCATIONERROR)
    {
        inourfield =  true;
    }
    else
    {
        inourfield =  false;
    }
    rtvl =  inourfield;

    return rtvl;
}

/* 是否在对方大禁区 */
bool FieldInformation::isOppPenalty(DPoint world_pt)
{
    bool rtvl = false;
    static bool inopppenalty = false;

    if(!inopppenalty)
    {
        if(world_pt.x_ > opp_penaltyarea_[1].x_ && world_pt.x_ < opp_penaltyarea_[0].x_ &&
           world_pt.y_ > opp_penaltyarea_[2].y_ && world_pt.y_ < opp_penaltyarea_[1].y_)
        {
            inopppenalty = true;
        }
        else
            inopppenalty = false;
    }
    else if(inopppenalty)
    {
        if(world_pt.x_ > (opp_penaltyarea_[1].x_ - simLOCATIONERROR) && world_pt.x_ < (opp_penaltyarea_[0].x_ + simLOCATIONERROR) &&
           world_pt.y_ > (opp_penaltyarea_[2].y_ - simLOCATIONERROR) && world_pt.y_ < (opp_penaltyarea_[1].y_ + simLOCATIONERROR))
        {
            inopppenalty = true;
        }
        else
            inopppenalty = false;
    }

    rtvl =  inopppenalty;
    return rtvl;
}

/* 是否在我方大禁区 */
bool FieldInformation::isOurPenalty(DPoint world_pt)
{
    bool rtvl = false;
    static bool inourpenalty = false;

    if(!inourpenalty)
    {
        if(world_pt.x_ < our_penaltyarea_[1].x_ && world_pt.x_ > our_penaltyarea_[0].x_ &&
           world_pt.y_ > our_penaltyarea_[2].y_ && world_pt.y_ < our_penaltyarea_[1].y_)
        {
            inourpenalty = true;
        }
        else
            inourpenalty = false;
    }
    else if(inourpenalty)
    {
        if(world_pt.x_ < (our_penaltyarea_[1].x_ +simLOCATIONERROR) && world_pt.x_ > (our_penaltyarea_[0].x_ -simLOCATIONERROR) &&
           world_pt.y_ > (our_penaltyarea_[2].y_-simLOCATIONERROR) &&  world_pt.y_ < (our_penaltyarea_[1].y_+simLOCATIONERROR))
        {
            inourpenalty = true;
        }
        else
            inourpenalty = false;
    }

    rtvl =  inourpenalty;
    return rtvl;
}

/* 是否在对方小禁区 */
bool FieldInformation::isOppGoal(DPoint world_pt)
{
    bool rtvl = false;
    static bool inopparea = false;

    if(!inopparea)
    {
        if(world_pt.x_ > opp_goalarea_[1].x_ && world_pt.x_ < opp_goalarea_[0].x_ &&
           world_pt.y_ > opp_goalarea_[2].y_ && world_pt.y_ < opp_goalarea_[1].y_)
        {
            inopparea = true;
        }
        else
            inopparea = false;
    }
    else if(inopparea)
    {
        if(world_pt.x_ > (opp_goalarea_[1].x_ - simLOCATIONERROR) && world_pt.x_ < (opp_goalarea_[0].x_ + simLOCATIONERROR) &&
           world_pt.y_ > (opp_goalarea_[2].y_-simLOCATIONERROR) && world_pt.y_ < (opp_goalarea_[1].y_+simLOCATIONERROR))
        {
            inopparea = true;
        }
        else
            inopparea = false;
    }

    rtvl =  inopparea;
    return rtvl;
}

/* 是否在我方小禁区 */
bool FieldInformation::isOurGoal(DPoint world_pt)
{
    bool rtvl = false;
    static bool inourarea = false;

    if(!inourarea)
    {
        if(world_pt.x_ < our_goalarea_[1].x_ && world_pt.x_ > our_goalarea_[0].x_ &&
           world_pt.y_ > our_goalarea_[2].y_ && world_pt.y_ < our_goalarea_[1].y_)
        {
            inourarea = true;
        }
        else
            inourarea = false;
    }
    else if(inourarea)
    {
        if(world_pt.x_ < (our_goalarea_[1].x_ +simLOCATIONERROR) && world_pt.x_ > (our_goalarea_[0].x_ -simLOCATIONERROR) &&
           world_pt.y_ > (our_goalarea_[2].y_-simLOCATIONERROR) &&  world_pt.y_ < (our_goalarea_[1].y_+simLOCATIONERROR))
        {
            inourarea = true;
        }
        else
            inourarea = false;
    }

    rtvl =  inourarea;
    return rtvl;
}

bool FieldInformation::isOutBorder(Border id, DPoint world_pt, double expand_len)
{
    switch (id)
    {
    case LEFTBORDER:
    {
        if(world_pt.x_ < xline_[6] - expand_len)    return 1;
        else    return 0;
        break;
    }
    case RIGHTBORDER:
    {
        if(world_pt.x_ > xline_[0] + expand_len)    return 1;
        else    return 0;
        break;
    }
    case UPBORDER:
    {
        if(world_pt.y_ > yline_[0] + expand_len)    return 1;
        else    return 0;
        break;
    }
    case DOWNBORDER:
    {
        if(world_pt.y_ < yline_[5] - expand_len)    return 1;
        else    return 0;
        break;
    }
    default:
    {
        return 1;
        break;
    }
    }
}
