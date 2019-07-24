#ifndef DRIBBLE_STATUS_HPP
#define DRIBBLE_STATUS_HPP
#include "core.hpp"
class DribbleState
{
public:
  bool is_dribble_;

  nubot::DPoint satrt_point_;

  DribbleState()
      :is_dribble_(false)
  {
  };

  void reset()
  {
      is_dribble_ = false;
  };

  void set(const nubot::DPoint &_pos_satrt)
  {
      is_dribble_ = true;
      satrt_point_ = _pos_satrt;
  };
  void update( const bool &_is_dribble, const nubot::DPoint &_pos_robot, const nubot::DPoint &_ball)
  {

      if( _is_dribble == is_dribble_ )
          return;
      if( _is_dribble)
         set(_pos_robot);
      else if( _ball.x_>50 || fabs(_ball.y_)>30)
          reset();
/*      if( _ball.x_>38 || fabs(_ball.y_)>30 )
          reset();
      else if( !is_dribble_ )
          set(_pos_robot);*/
  };

  nubot::DPoint limitWithinCircle( const nubot::DPoint &_point_in, const int _radius = 300)
  {
      if( (_point_in-satrt_point_).length()<_radius )
		  return _point_in;
	  else
          return satrt_point_ + (_point_in-satrt_point_) *( _radius/(_point_in-satrt_point_).length());
  };
};

#endif
