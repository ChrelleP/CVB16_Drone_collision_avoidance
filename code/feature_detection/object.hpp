//-------------------------------------------------------------
// Objects - CVB16
//
// This class is used to define objects found in images.
//
// Made by: Christian
// ------------------------------------------------------------

#pragma once

#include <iostream>
#include <math.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;

class object
{
  public:
    object();
    object(Vec2f first_line, Vec2f second_line);

    bool update(Vec2f &line);

    bool alive_decount();

    float max(float &a, float &b);
    float min(float &a, float &b);
    float max_abs(float &a, float &b);
    float min_abs(float &a, float &b);

    bool closest_feature(Vec2f &line);    // True = Upper | False = Lower

    float re_angle();
    float re_center();
    float re_width();

    bool half_speed;
    bool full_stop;
    bool feecback;

    float calc_distance();

    void collison_risk();

  	~object();

  private:
    float width;
    float center;

    float lower_angle;
    float upper_angle;

    float lower_feature;
    float upper_feature;

    float angle;
    int alive_count_upper;
    int alive_count_lower;
};
