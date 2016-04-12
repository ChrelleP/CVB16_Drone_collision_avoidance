//-------------------------------------------------------------
// Feature Detection - CVB16
//
// This class is used to detect features in a video,
// specifically straight bars in various angles and size.
//
// Input:    A video.
// Output:   A video showing the lines in the picture
//           or a vector of objects describing the bars
//           in the image.
//
// Made by: Christian and Vincent
// ------------------------------------------------------------

#pragma once

#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "object.hpp"
#include <math.h>

// DEFINES
#define REACT_STOP           1
#define REACT_FEEDBACK       2
#define REACT_LEFT           3
#define REACT_HALFSPEED      4

using namespace std;
using namespace cv;

class feature_detection
{
  public:
    feature_detection();
	  feature_detection(const char* file);

    Mat source;

    void filter(int &lb, int &ub, int &as);
    void edges(int &lb, int &ub, int &as);
    void lines(float &rho, float &theta, int &threshold);

    void show_source();
    void show_filter();
    void show_edge_map();

    void draw_lines();
    void draw_filtered_lines();
    void draw_objects();

    void filter_houghlines();

    float dst_hspace(Vec2f first, Vec2f second);

    void identify_objects();

    float calc_distance();

    int collison_risk(int global_react);

  	~feature_detection();

  private:
    const char* filename;

    Mat filtered;
    Mat edge_map;

    bool no_error;

    vector<Vec2f> hough_lines;
    vector<Vec2f> filtered_lines;

    vector<object> bars;
    vector<float> distances;
};
