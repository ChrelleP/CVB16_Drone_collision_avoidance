////////////////////////////////////////////////////////////////////////////////
// MAIN FILE FOR UAV DRONE
//
// Made by: Christian and Vincent
////////////////////////////////////////////////////////////////////////////////

// ---------------------------- INCLUDES ---------------------------------------
#include <cstdlib>
#include <pthread.h>
#include <iostream>
#include <fstream>
#include "stdio.h"
#include "wiringSerial.h"
#include "feature_detection.hpp"
#include "object.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "DSM_analyser.hpp"

using namespace std;
using namespace cv;

// ----------------------------- DEFINES ---------------------------------------

#define STATE_FEEDBACK       1
#define STATE_STOP           2
#define STATE_AVOID          3

#define REACT_STOP           1
#define REACT_FEEDBACK       2
#define REACT_LEFT           3

// ------------------------------ STRUCTS --------------------------------------
struct data_packet{
  int throttle;
  int yaw;
  int pitch;
  int rotate;
};

// --------------------------- GLOBAL VARIABLES --------------------------------

volatile int global_reaction;

pthread_mutex_t reaction_mutex = PTHREAD_MUTEX_INITIALIZER;

// --------------------------- FUNCTIONS ---------------------------------------
void *CV_avoid(void *arg)
{
   //------------- Create objects and variables -------------
   feature_detection FT;   // Feature Detection object - used for CV methods
   VideoCapture cap(0);       // Video Capture object - used to get frames from video

   //------------- Variables --------------------------------

   int LB_MASK = 65;                 // Lower bound for mask
   int UB_MASK = 98;                 // Upper bound for mask
   int AS_MEDIAN = 7;                  // Apperture size for median filter

   int LB_CANNY = 150;                // Lower bound for canny
   int UB_CANNY = 200;                // Upper bound for canny
   int AS_CANNY = 3;                  // Apperture size for canny filter

   float RHO = 1;                  // Rho used for HoughTransform
   float THETA = 3.1416/245;         // Theta used for HoughTransform
   int THRESHOLD = 110;                // Threshold for HoughTransform

   //------------------ While loop ---------------------------
   while(true)
   {
     bool success = cap.read(FT.source);           // read a new frame from video

     if (!success)                                 //if not success, break loop
     {
       cout << "Could not read frame" << endl;
       break;
     }

     FT.filter(LB_MASK, UB_MASK, AS_MEDIAN);         // Filter the image
     FT.edges(LB_CANNY, UB_CANNY, AS_CANNY);         // Find edges with canny
     FT.lines(RHO, THETA, THRESHOLD);                // Find lines through Hough

     FT.filter_houghlines();
     FT.identify_objects();

     //FT.draw_objects();

     //FT.show_source();
     //FT.show_filter();
     //FT.show_edge_map();

     if(waitKey(10) == 27)                         // Wait 50 ms untill next frame, exit if escape is pressed
     {
       cout << "esc key is pressed by user" << endl;
       break;
     }
   }

   pthread_exit(NULL);
}

int main ()
{
  // -------- Open Serial ---------
  int fd; // File descriptor for AMA0

  if ((fd = serialOpen ("/dev/ttyAMA0", 115200)) < 0)
  {
    printf ("Unable to open serial device") ;
  }

  serialFlush(fd);

  // -------- Start threads -------
  /*pthread_t CV_thread;
  int CV_rc;

  CV_rc = pthread_create( &CV_thread, NULL, CV_avoid, NULL);
  if( CV_rc )
    printf("Thread creation failed: %d\n", CV_rc);
  */
  // ------ Variables -------------
  int state = STATE_FEEDBACK;
  int local_reaction;

  bool abort = false;

  data_packet RX;
  data_packet TX;

  int temp_byte;
  int lowbyte;
  int highbyte;

  // ------- Main loop -----------
  while(abort)
  {
    // --------- Recieve -----------

    // --------- State machine ----------
    // Retrieve reaction
    pthread_mutex_lock( &reaction_mutex );
    local_reaction = global_reaction;
    pthread_mutex_unlock( &reaction_mutex );

    // Switch on the state
    switch(state)
    {
      case STATE_FEEDBACK:
          // _________ FEEDBACK STATE _____________
          // RX -> TX

          TX.throttle = RX.throttle;
          TX.pitch = RX.pitch;
          //...

          // Update state

          switch(local_reaction)
          {
            case REACT_STOP: state = STATE_STOP;
                             break;
            case REACT_LEFT: state = STATE_STOP;
                             break;
            default: cout << "Error in state change" << endl;
                          break;
          }
          break;
      case STATE_STOP:
          // _________ STOP STATE _________________
          // Keep constant throttle, everything else 0.

          TX.throttle = RX.throttle;

          // Update state
          switch(local_reaction)
          {
            case REACT_STOP: state = STATE_STOP;
                             break;
            case REACT_LEFT: state = STATE_STOP;
                             break;
            default: cout << "Error in state change" << endl;
                          break;
          }
          break;
       default: cout << "Error in state" << endl;
                     break;
    }
  }

  //pthread_join( CV_thread, NULL);

  return 0;
}
