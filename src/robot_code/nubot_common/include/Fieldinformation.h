#ifndef __NUBOT_NUBOTCONTROL_FIELDINFOMATION_H_
#define __NUBOT_NUBOTCONTROL_FIELDINFOMATION_H_

#include "core.hpp"

namespace nubot
{
using std::vector;
using std::string;

class FieldInformation
{
public:
    std::vector<int> xline_;
    std::vector<int> yline_;

    std::vector<Circle >     postcircle_;
    std::vector<LineSegment> x_white_line_;
    std::vector<LineSegment> y_white_line_;

    DPoint oppGoal_[5];
    DPoint ourGoal_[5];
    Circle centercircle_;
    DPoint ourPenaltyPt_;
    DPoint oppPenaltyPt_;
    LineSegment  ourPenaltyLine_[3];
    LineSegment  oppPenaltyLine_[3];

    /// 大禁区四个角点
    DPoint opp_penaltyarea_[4];
    DPoint our_penaltyarea_[4];
    DPoint our_goalarea_[4];
    DPoint opp_goalarea_[4];

    FieldInformation()
    {
        xline_.push_back(FIELD_XLINE1); /** 900，0*/
        xline_.push_back(FIELD_XLINE2);
        xline_.push_back(FIELD_XLINE3);
        xline_.push_back(FIELD_XLINE4);
        xline_.push_back(FIELD_XLINE5);
        xline_.push_back(FIELD_XLINE6);
        xline_.push_back(FIELD_XLINE7); /** -900，6*/

        yline_.push_back(FIELD_YLINE1); /** 600，0 */
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
#if defined(MATCH)
        oppGoal_[GOAL_UPPER]    = DPoint(xline_[0],100);
        oppGoal_[GOAL_MIDUPPER] = DPoint(xline_[0],40);
        oppGoal_[GOAL_MIDDLE]   = DPoint(xline_[0],0);
        oppGoal_[GOAL_MIDLOWER] = DPoint(xline_[0],-40);
        oppGoal_[GOAL_LOWER]    = DPoint(xline_[0],-100);

        ourGoal_[GOAL_UPPER]    = DPoint(xline_[6],100);
        ourGoal_[GOAL_MIDUPPER] = DPoint(xline_[6],40);
        ourGoal_[GOAL_MIDDLE]   = DPoint(xline_[6],0);
        ourGoal_[GOAL_MIDLOWER] = DPoint(xline_[6],-40);
        ourGoal_[GOAL_LOWER]    = DPoint(xline_[6],-100);
#else
//        oppGoal_[GOAL_UPPER]    = DPoint(xline_[0],80);
//        oppGoal_[GOAL_MIDUPPER] = DPoint(xline_[0],40);
//        oppGoal_[GOAL_MIDDLE]   = DPoint(xline_[0],0);
//        oppGoal_[GOAL_MIDLOWER] = DPoint(xline_[0],-40);
//        oppGoal_[GOAL_LOWER]    = DPoint(xline_[0],-80);

//        ourGoal_[GOAL_UPPER]    = DPoint(xline_[6],80);
//        ourGoal_[GOAL_MIDUPPER] = DPoint(xline_[6],40);
//        ourGoal_[GOAL_MIDDLE]   = DPoint(xline_[6],0);
//        ourGoal_[GOAL_MIDLOWER] = DPoint(xline_[6],-40);
//        ourGoal_[GOAL_LOWER]    = DPoint(xline_[6],-80);
        oppGoal_[GOAL_UPPER]    = DPoint(xline_[0],100);
        oppGoal_[GOAL_MIDUPPER] = DPoint(xline_[0],40);
        oppGoal_[GOAL_MIDDLE]   = DPoint(xline_[0],0);
        oppGoal_[GOAL_MIDLOWER] = DPoint(xline_[0],-40);
        oppGoal_[GOAL_LOWER]    = DPoint(xline_[0],-100);

        ourGoal_[GOAL_UPPER]    = DPoint(xline_[6],100);
        ourGoal_[GOAL_MIDUPPER] = DPoint(xline_[6],40);
        ourGoal_[GOAL_MIDDLE]   = DPoint(xline_[6],0);
        ourGoal_[GOAL_MIDLOWER] = DPoint(xline_[6],-40);
        ourGoal_[GOAL_LOWER]    = DPoint(xline_[6],-100);
#endif

        /// 我方大禁区
        our_penaltyarea_[0] = DPoint(xline_[6],yline_[1]);  //(-900,325)
        our_penaltyarea_[1] = DPoint(xline_[4],yline_[1]);  //(-675,325)
        our_penaltyarea_[2] = DPoint(xline_[4],yline_[4]);  //(-675,-325)
        our_penaltyarea_[3] = DPoint(xline_[6],yline_[4]);  //(-900,-325)
        ourPenaltyLine_[0] = LineSegment(our_penaltyarea_[0], our_penaltyarea_[1]);
        ourPenaltyLine_[1] = LineSegment(our_penaltyarea_[1], our_penaltyarea_[2]);
        ourPenaltyLine_[2] = LineSegment(our_penaltyarea_[2], our_penaltyarea_[3]);

        /// 对方大禁区
        opp_penaltyarea_[0] = DPoint(xline_[0],yline_[1]);
        opp_penaltyarea_[1] = DPoint(xline_[2],yline_[1]);
        opp_penaltyarea_[2] = DPoint(xline_[2],yline_[4]);
        opp_penaltyarea_[3] = DPoint(xline_[0],yline_[4]);
        oppPenaltyLine_[0] = LineSegment(opp_penaltyarea_[0], opp_penaltyarea_[1]);
        oppPenaltyLine_[1] = LineSegment(opp_penaltyarea_[1], opp_penaltyarea_[2]);
        oppPenaltyLine_[2] = LineSegment(opp_penaltyarea_[2], opp_penaltyarea_[3]);

        /// 双方小禁区
        our_goalarea_[0] = DPoint(xline_[6],yline_[2]);   //(-900,175)
        our_goalarea_[1] = DPoint(xline_[5],yline_[2]);   //(-825,175)
        our_goalarea_[2] = DPoint(xline_[5],yline_[3]);   //(-825,-175)
        our_goalarea_[3] = DPoint(xline_[6],yline_[3]);   //(-900,-175)

        opp_goalarea_[0] = DPoint(xline_[0],yline_[2]);   //(900,175)
        opp_goalarea_[1] = DPoint(xline_[1],yline_[2]);   //(825,175)
        opp_goalarea_[2] = DPoint(xline_[1],yline_[3]);   //(825,-175)
        opp_goalarea_[3] = DPoint(xline_[0],yline_[3]);   //(900,-175)

        ourPenaltyPt_ = DPoint(-750,0);
        oppPenaltyPt_ = DPoint(750,0);
    }

    FieldInformation(string infopath)
    {

    }

    /* 是否在场地内 */
    bool isInInterField(DPoint world_pt,double expand_len=0)
    {
        return (world_pt.x_>xline_[6]-expand_len &&
                world_pt.x_<xline_[0]+expand_len &&
                world_pt.y_>yline_[5]-expand_len &&
                world_pt.y_<yline_[0]+expand_len);
    }

    bool isInInterField2(DPoint world_pt, double expand_len=0, double expand_width=0)
    {
        return (world_pt.x_>xline_[6]-expand_len &&
                world_pt.x_<xline_[0]+expand_len &&
                world_pt.y_>yline_[5]-expand_width&&
                world_pt.y_<yline_[0]+expand_width);
    }

    /* 是否在场地外 */
    bool isInOuterField(DPoint world_pt,double expand_len=0)
    {
        return (world_pt.x_<xline_[6]-expand_len ||
                world_pt.x_>xline_[0]+expand_len ||
                world_pt.y_<yline_[5]-expand_len ||
                world_pt.y_>yline_[0]+expand_len);
    }

    /* 是否在对方半场 */
    bool isOppField(DPoint world_pt)
    {
        bool rtvl = false;
        static bool inoppfield = false;

        if(!inoppfield && world_pt.x_ > 0)
        {
            inoppfield  =  true;
        }
        else if(inoppfield && world_pt.x_ >  -LOCATIONERROR)
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
    bool isOurField(DPoint world_pt)
    {
        bool rtvl = false;
        static bool inourfield = false;

        if(!inourfield && world_pt.x_ < 0)
        {
            inourfield  =  true;
        }
        else if(inourfield && world_pt.x_ < LOCATIONERROR)
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
    bool isOppPenalty(DPoint world_pt)
    {
        bool rtvl = false;
        static bool inopppenalty = false;

        if(!inopppenalty)
        {
            if(world_pt.x_ > opp_penaltyarea_[1].x_ &&
                    world_pt.y_ > opp_penaltyarea_[2].y_ &&
                    world_pt.y_ < opp_penaltyarea_[1].y_)
            {
                inopppenalty = true;
            }
            else
                inopppenalty = false;
        }
        else if(inopppenalty)
        {
            if(world_pt.x_ > (opp_penaltyarea_[1].x_ - LOCATIONERROR) &&
                    world_pt.y_ > (opp_penaltyarea_[2].y_ - LOCATIONERROR) &&
                    world_pt.y_ < (opp_penaltyarea_[1].y_ + LOCATIONERROR))
            {
                inopppenalty = true;
            }
            else
                inopppenalty = false;
        }

        rtvl =  inopppenalty;
        return rtvl;
    }

    /* 是否在我方大禁区 expand_len */
    bool isOurPenalty_expand(DPoint world_pt, double expand_len=0)
    {
        bool rtvl = false;
        static bool inourpenalty = false;

        if(!inourpenalty)
        {
            if(world_pt.x_ < our_penaltyarea_[1].x_ + expand_len &&
                    world_pt.y_ > our_penaltyarea_[2].y_  - expand_len &&
                    world_pt.y_ < our_penaltyarea_[1].y_ + expand_len)
            {
                inourpenalty = true;
            }
            else
                inourpenalty = false;
        }
        else if(inourpenalty)
        {
            if(world_pt.x_ < (our_penaltyarea_[1].x_ +LOCATIONERROR) &&
                    world_pt.y_ > (our_penaltyarea_[2].y_-LOCATIONERROR) &&
                    world_pt.y_ < (our_penaltyarea_[1].y_+LOCATIONERROR))
            {
                inourpenalty = true;
            }
            else
                inourpenalty = false;
        }

        rtvl =  inourpenalty;
        return rtvl;
    }

    /* 是否在我方大禁区 */
    bool isOurPenalty(DPoint world_pt)
    {
        bool rtvl = false;
        static bool inourpenalty = false;

        if(!inourpenalty)
        {
            if(world_pt.x_ < our_penaltyarea_[1].x_ &&
                    world_pt.y_ > our_penaltyarea_[2].y_ &&
                    world_pt.y_ < our_penaltyarea_[1].y_)
            {
                inourpenalty = true;
            }
            else
                inourpenalty = false;
        }
        else if(inourpenalty)
        {
            if(world_pt.x_ < (our_penaltyarea_[1].x_ +LOCATIONERROR) &&
                    world_pt.y_ > (our_penaltyarea_[2].y_-LOCATIONERROR) &&
                    world_pt.y_ < (our_penaltyarea_[1].y_+LOCATIONERROR))
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
    bool isOppGoal(DPoint world_pt)
    {
        bool rtvl = false;
        static bool inopparea = false;

        if(!inopparea)
        {
            if(world_pt.x_ > opp_goalarea_[1].x_ &&
                    world_pt.y_ > opp_goalarea_[2].y_ &&
                    world_pt.y_ < opp_goalarea_[1].y_)
            {
                inopparea = true;
            }
            else
                inopparea = false;
        }
        else if(inopparea)
        {
            if(world_pt.x_ > (opp_goalarea_[1].x_ - LOCATIONERROR) &&
                    world_pt.y_ > (opp_goalarea_[2].y_-LOCATIONERROR) &&
                    world_pt.y_ < (opp_goalarea_[1].y_+LOCATIONERROR))
            {
                inopparea = true;
            }
            else
                inopparea = false;
        }

        rtvl =  inopparea;
        return rtvl;
    }

    bool isOutBorder(Border id, DPoint world_pt, double expand_len=0)
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

    /* 是否在我方小禁区 */
    bool isOurGoal(DPoint world_pt)
    {
        bool rtvl = false;
        static bool inourarea = false;

        if(!inourarea)
        {
            if(world_pt.x_ < our_goalarea_[1].x_ &&
                    world_pt.y_ > our_goalarea_[2].y_ &&
                    world_pt.y_ < our_goalarea_[1].y_)
            {
                inourarea = true;
            }
            else
                inourarea = false;
        }
        else if(inourarea)
        {
            if(world_pt.x_ < (our_goalarea_[1].x_ +LOCATIONERROR) &&
                    world_pt.y_ > (our_goalarea_[2].y_-LOCATIONERROR) &&
                    world_pt.y_ < (our_goalarea_[1].y_+LOCATIONERROR))
            {
                inourarea = true;
            }
            else
                inourarea = false;
        }

        rtvl =  inourarea;
        return rtvl;
    }
};

}
#endif  //!__NUBOT_VISION_FIELDINFOMATION_H_

