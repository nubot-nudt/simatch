#ifndef __NUBOT_NUBOTCONTROL_FIELDINFOMATION_H_
#define __NUBOT_NUBOTCONTROL_FIELDINFOMATION_H_

#include <opencv2/opencv.hpp>
#include "nubot/core/core.hpp"

namespace nubot
{
using std::vector;
using std::string;

enum GoalLocation
{
     GOAL_UPPER     = 0,
     GOAL_MIDUPPER  = 1,
     GOAL_MIDDLE    = 2,
     GOAL_MIDLOWER  = 3,
     GOAL_LOWER     = 4,
};

class FieldInformation
{
public:

    FieldInformation();
    FieldInformation(string infopath);

    bool isInInterField(DPoint world_pt, double expand_len=0);
    bool isInInterField2(DPoint world_pt, double expand_len=0, double expand_width=0);   // len只对场地的长度作限制
    bool isInOuterField(DPoint world_pt, double expand_len=0);
    bool isOppField(DPoint world_pt);
    bool isOurField(DPoint world_pt);
    bool isOppPenalty(DPoint world_pt);     //是否在我方（对方）大禁区
    bool isOurPenalty(DPoint world_pt);
    bool isOurGoal(DPoint world_pt);        //是否在我方（对方）小禁区
    bool isOppGoal(DPoint world_pt);
    bool isOutBorder(Border id, DPoint world_pt, double expand_len=0);   // 是否超出边界

    std::vector<int> xline_;
    std::vector<int> yline_;

    std::vector<Circle > postcircle_;
    std::vector<LineSegment> x_white_line_;
    std::vector<LineSegment> y_white_line_;

    DPoint oppGoal_[5]; /** 5 the number of GoalLocation */
    DPoint ourGoal_[5];
    Circle centercircle_;
    DPoint ourPenaltyPt_;              // 罚球点
    DPoint oppPenaltyPt_;
    LineSegment  ourPenaltyLine_[3];
    LineSegment  oppPenaltyLine_[3];

    DPoint opp_penaltyarea_[4];        //大禁区四个角点
    DPoint our_penaltyarea_[4];
    DPoint our_goalarea_[4];
    DPoint opp_goalarea_[4];
};



}
#endif  //!__NUBOT_VISION_FIELDINFOMATION_H_

