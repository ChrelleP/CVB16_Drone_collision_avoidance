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

    float re_angle();
    float re_center();
    float re_width();

  	~object();

  private:
    float width;
    float center;

    float lower_angle;
    float upper_angle;

    float lower_feature;
    float upper_feature;

    float angle;
    int alive_count;
};
