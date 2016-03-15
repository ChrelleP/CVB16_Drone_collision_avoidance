#include "feature_detection.hpp"

object::object()
{
}

object::object(Vec2f first_line, Vec2f second_line)
{
  cout << "fl: " << first_line[0] << " sl: " << second_line[0] << endl;
  cout << "fa: " << first_line[1] << " sa: " << second_line[1] << endl;

  lower_angle = min(first_line[1], second_line[1]);
  upper_angle = max(first_line[1], second_line[1]);
  angle = (upper_angle + lower_angle)/2;

  lower_feature = min(first_line[0], second_line[0]);
  upper_feature = max(first_line[0], second_line[0]);
  width = abs(upper_feature) - abs(lower_feature);

  if(lower_feature < 0)
    center = lower_feature - (width / 2);
  else
    center = lower_feature + (width / 2);

  if(angle >= 0 && angle <= M_PI/2)
    center = abs(center);

  alive_count = 10;

  cout << "---New object---" << endl;
  cout << "angle: " << angle << endl;
  cout << "center: " << center << endl;
  cout << "width: " << width << endl;
}

bool object::update(Vec2f &line)
{
  float diff_rho1 = line[0] - lower_feature;
  float diff_rho2 = line[0] - upper_feature;
  float diff_theta = line[1] - angle;

  if( diff_theta < 0.087 && diff_theta > -0.087 )
  {
    if( diff_rho1 < 10 && diff_rho1 > -10 )
    {
      alive_count = 10;
      angle = line[1];
      lower_feature = line[0];
      width = abs(upper_feature) - abs(lower_feature);

      if(lower_feature < 0)
        center = lower_feature - (width / 2);
      else
        center = lower_feature + (width / 2);

      return true;
    }
    else if( diff_rho2 < 10 && diff_rho2 > -10 )
    {
      alive_count = 10;
      angle = line[1];
      upper_feature = line[0];
      width = abs(upper_feature) - abs(lower_feature);

      if(lower_feature < 0)
        center = lower_feature - (width / 2);
      else
        center = lower_feature + (width / 2);

      return true;
    }
  }
  else
    return false;
}

bool object::alive_decount()
{
  if(alive_count > 0)
  {
    alive_count--;
    return false;
  }
  else
    return true;
}

float object::max(float &a, float &b)
{
  if( abs(a) < abs(b) )
    return b;
  else
    return a;
}

float object::min(float &a, float &b)
{
  if( abs(a) > abs(b) )
    return b;
  else
    return a;
}

float object::re_angle()
{
  return angle;
}

float object::re_width()
{
  return width;
}

float object::re_center()
{
    return center;
}

object::~object()
{

}
