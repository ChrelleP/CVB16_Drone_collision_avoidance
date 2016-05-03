#include <iostream>
#include <fstream>
#include "feature_detection.hpp"
//#include "object.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
  //----------------- Get video filename ------------------
 const char* filename;

  if(argc == 2)
    filename = argv[1];
  else{
    cout << "Specify a name for video" << endl;
    return 0;
  }
  //--------------------------------------------------------

  //------------- Create objects and variables -------------
  feature_detection FT(filename);   // Feature Detection object - used for CV methods
  VideoCapture cap(filename);       // Video Capture object - used to get frames from video

  int lb_mask = 60;                 // Lower bound for mask
  int ub_mask = 98;                // Upper bound for mask
  int as_median = 7;                // Apperture size for median filter

  int lb_canny = 150;               // Lower bound for canny
  int ub_canny = 200;               // Upper bound for canny
  int as_canny = 3;                 // Apperture size for canny filter

  float rho = 1;                    // Rho used for HoughTransform
  float theta = 3.1416/245;         // Theta used for HoughTransform
  int threshold = 110;              // Threshold for HoughTransform
  //---------------------------------------------------------

  //------------------ Main while loop ----------------------
  while(true)
  {
    bool success = cap.read(FT.source);           // read a new frame from video
    /*
    const int kNewWidth = 600;
    const int kNewHeight = 600;

    const float kScaleFactor = 0.50;

    //resize(FT.source, FT.source, cvSize(0, 0), kScaleFactor, kScaleFactor);
    */
    if (!success)                                 //if not success, break loop
    {
      cout << "Could not read frame" << endl;
      break;
    }

    FT.filter(lb_mask,ub_mask,as_median);         // Filter the image
    FT.edges(lb_canny,ub_canny,as_canny);         // Find edges with canny
    FT.lines(rho,theta,threshold);                // Find lines through Hough

    FT.filter_houghlines();
    FT.identify_objects();

    FT.draw_objects();
    FT.draw_filtered_lines();
    //FT.draw_lines();

    FT.show_source();
    FT.show_filter();
    //FT.show_edge_map();

/*
    ofstream myfile;
    myfile.open ("Hough_Lines.txt");
    myfile << getHougLines();
    myfile.close();
*/

    if(waitKey(100) == 27)                         // Wait 50 ms untill next frame, exit if escape is pressed
    {
      cout << "esc key is pressed by user" << endl;
      break;
    }
   }
   //---------------------------------------------------------
}
