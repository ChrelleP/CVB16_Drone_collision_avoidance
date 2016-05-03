#include "feature_detection.hpp"

object::object()
{
}

object::object(Vec2f first_line, Vec2f second_line)
{
  //cout << "fl: " << first_line[0] << " sl: " << second_line[0] << endl;
  //cout << "fa: " << first_line[1] << " sa: " << second_line[1] << endl;

  lower_angle = min(first_line[1], second_line[1]);
  upper_angle = max(first_line[1], second_line[1]);
  angle = (upper_angle + lower_angle)/2;

  lower_feature = min_abs(first_line[0], second_line[0]);
  upper_feature = max_abs(first_line[0], second_line[0]);
  width = abs(upper_feature) - abs(lower_feature);

  if(lower_feature < 0)
    center = lower_feature - (width / 2);
  else
    center = lower_feature + (width / 2);

  if(angle >= 0 && angle <= M_PI/2)
    center = abs(center);

  alive_count_upper = 10;
  alive_count_lower = 10;

  /*
  cout << "---New object---" << endl;
  cout << "angle: " << angle << endl;
  cout << "center: " << center << endl;
  cout << "width: " << width << endl;
  */
}

bool object::update(Vec2f &line)
{
  bool cl_feat = closest_feature(line);
  float diff_rho;

  if(cl_feat)
    diff_rho = line[0] - upper_feature;
  else
    diff_rho = line[0] - lower_feature;

  float diff_theta = line[1] - angle;

  if( abs(diff_theta) < 0.087 )
  {
    if( abs(diff_rho) < 10 )
    {
      alive_count_lower = 10;
      angle = line[1];

      if(cl_feat)
        upper_feature = line[0];
      else
        lower_feature = line[0];

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
  if(alive_count_lower > 0 && alive_count_upper > 0)
  {
    alive_count_lower--;
    alive_count_upper--;
    return false;
  }
  else
    return true;
}

float object::max_abs(float &a, float &b)
{
  if( abs(a) < abs(b) )
    return b;
  else
    return a;
}

float object::min_abs(float &a, float &b)
{
  if( abs(a) > abs(b) )
    return b;
  else
    return a;
}

float object::max(float &a, float &b)
{
  if( a < b )
    return b;
  else
    return a;
}

float object::min(float &a, float &b)
{
  if( a > b )
    return b;
  else
    return a;
}

bool object::closest_feature(Vec2f &line)
{
  float diff_lower = abs(line[0] - lower_feature);
  float diff_upper = abs(line[0] - upper_feature);

  if(diff_lower < diff_upper)
    return false;
  else
    return true;
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
float object::calc_distance()
{

}
void object::collison_risk()
{
  /*
  if(distance <= 4 && !full_stop)
  {
    half_speed = true;
    if(distance <= 2)
    {
      full_stop = true;
    }
    else
    full_stop = false;
  }
  else
  {
    if(full_stop)
    {
      half_speed = true;

    }
    else
    feecback = true;
    half_speed = false;
  }
  */
}
object::~object()
{

}
