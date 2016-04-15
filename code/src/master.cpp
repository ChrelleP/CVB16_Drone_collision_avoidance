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
#include "wiringPi.h"

using namespace std;
using namespace cv;

// ----------------------------- DEFINES ---------------------------------------

#define STATE_FEEDBACK       1
#define STATE_STOP           2
#define STATE_AVOID          3

#define REACT_NOTHING        0
#define REACT_STOP           1
#define REACT_FEEDBACK       2
#define REACT_LEFT           3
#define REACT_HALFSPEED      4

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
   int local_reaction = REACT_NOTHING;

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


     pthread_mutex_lock( &reaction_mutex );
     local_reaction = global_reaction;
     pthread_mutex_unlock( &reaction_mutex );

     local_reaction = FT.collison_risk(local_reaction);

     pthread_mutex_lock( &reaction_mutex );
     global_reaction = local_reaction;
     pthread_mutex_unlock( &reaction_mutex );

     FT.draw_objects();

     FT.show_source();
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
  wiringPiSetup () ;
  pinMode (16, OUTPUT) ;
  for (;;)
  {
    digitalWrite (16, HIGH) ; delay (500) ;
    digitalWrite (16,  LOW) ; delay (500) ;
  }




  // -------- Startup ---------
  DSM_RX_TX DSM_UART;

  pthread_t CV_thread;
  int CV_rc;

  CV_rc = pthread_create( &CV_thread, NULL, CV_avoid, NULL);
  if( CV_rc )
    printf("Thread creation failed: %d\n", CV_rc);

  // ------ Variables -------------
  int state = STATE_STOP;
  int local_reaction = REACT_NOTHING;

  bool abort = false;

  package RX;
  package TX;

  TX.channel_value[0] = CHANNEL0_DEFAULT;
  TX.channel_value[1] = CHANNEL1_DEFAULT;
  TX.channel_value[2] = CHANNEL2_DEFAULT;
  TX.channel_value[3] = CHANNEL3_DEFAULT;
  TX.channel_value[4] = CHANNEL4_DEFAULT;
  TX.channel_value[5] = CHANNEL5_DEFAULT;
  TX.channel_value[6] = CHANNEL6_DEFAULT;

  // ------- Main loop -----------
  while(!abort)
  {
    // --------- Recieve -----------
    RX = DSM_UART.DSM_analyse(false, TX);

    printf("------- Transmitted -------\n");
    printf("channel 0 = %d \n", TX.channel_value[0]);
    printf("channel 1 = %d \n", TX.channel_value[1]);
    printf("channel 2 = %d \n", TX.channel_value[2]);
    printf("channel 3 = %d \n", TX.channel_value[3]);
    printf("channel 4 = %d \n", TX.channel_value[4]);
    printf("channel 5 = %d \n", TX.channel_value[5]);
    printf("channel 6 = %d \n", TX.channel_value[6]);

    printf("------- Received ----------\n");
    printf("channel 0 = %d \n", RX.channel_value[0]);
    printf("channel 1 = %d \n", RX.channel_value[1]);
    printf("channel 2 = %d \n", RX.channel_value[2]);
    printf("channel 3 = %d \n", RX.channel_value[3]);
    printf("channel 4 = %d \n", RX.channel_value[4]);
    printf("channel 5 = %d \n", RX.channel_value[5]);
    printf("channel 6 = %d \n", RX.channel_value[6]);

    system("clear");

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
          TX = RX;

          // Update state
          switch(local_reaction)
          {
            case REACT_STOP: state = STATE_STOP;
                             break;
            case REACT_LEFT: state = STATE_STOP;
                             break;
            default:         break; // No state change
          }
          break;
      case STATE_STOP:
          // _________ STOP STATE _________________
          // Keep constant throttle, everything else 0.
          TX = RX;

          TX.channel_value[6] = 0;

          // Update state
          switch(local_reaction)
          {
            case REACT_STOP: state = STATE_STOP;
                             break;
            case REACT_LEFT: state = STATE_STOP;
                             break;
            default:         break;
          }
          break;
       default: cout << "Error in state" << endl;
                abort = true;
                     break;
    }
  }

  DSM_UART.DSM_analyse(true, RX);
  //pthread_join( CV_thread, NULL);

  return 0;
}
