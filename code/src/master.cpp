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

#define STATE_STOP           1
#define STATE_FEEDBACK       2
#define STATE_HALFSPEED      3
#define STATE_CHANGING       4

#define REACT_NOTHING        0
#define REACT_STOP           1
#define REACT_FEEDBACK       2
#define REACT_HALFSPEED      3
#define REACT_LEFT           4
#define REACT_RIGHT          5

#define THROTTLE             0
#define YAW                  3      // Right -- | Left ++
#define PITCH                2      // Forward -- | Backwards ++
#define ROLL                 1      // Left -- | Right ++
#define FLIGHT_MODE          4      // Mode 0: 1704 | Mode 1: 1192 | Mode 2: 340
#define PANIC_BIND           5

// --------------------------- GLOBAL VARIABLES --------------------------------

volatile int global_reaction;

pthread_mutex_t reaction_mutex = PTHREAD_MUTEX_INITIALIZER;

// --------------------------- FUNCTIONS ---------------------------------------
void *CV_avoid(void *arg)
{
   //------------- Create objects and variables -------------
   feature_detection FT;   // Feature Detection object - used for CV methods
   VideoCapture cap("FVS_3.avi");       // Video Capture object - used to get frames from video

   //------------- Variables --------------------------------
   int local_reaction = REACT_NOTHING;
   int temp_reaction = REACT_NOTHING;

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
     temp_reaction = global_reaction;
     pthread_mutex_unlock( &reaction_mutex );

     local_reaction = FT.collision_risk(temp_reaction);

     pthread_mutex_lock( &reaction_mutex );
     global_reaction = local_reaction;
     pthread_mutex_unlock( &reaction_mutex );

     //FT.draw_objects();

     //FT.show_source();
     //FT.show_filter();
     //FT.show_edge_map();

     if(waitKey(33) == 27)                         // Wait 50 ms untill next frame, exit if escape is pressed
     {
       cout << "esc key is pressed by user" << endl;
       break;
     }
   }

   pthread_exit(NULL);
}

int main ()
{
  // -------- Startup ---------
  DSM_RX_TX DSM_UART;

  pthread_t CV_thread;
  int CV_rc;

  //CV_rc = pthread_create( &CV_thread, NULL, CV_avoid, NULL);
  //if( CV_rc )
    //printf("Thread creation failed: %d\n", CV_rc);

  // ------ Variables -------------
  int state = STATE_FEEDBACK;
  int local_reaction = REACT_NOTHING;
  int stop_value = 0;
  int packet_counter = 0;
  int temp_FM;

  int throttle_default = CHANNEL0_DEFAULT;
  int yaw_default = CHANNEL3_DEFAULT;
  int pitch_default = CHANNEL2_DEFAULT;
  int roll_default = CHANNEL1_DEFAULT;

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

  RX = DSM_UART.DSM_analyse(false, TX); // Receive a single frame for finding default values.

  throttle_default = RX.channel_value[THROTTLE];
  yaw_default = RX.channel_value[YAW];
  pitch_default = RX.channel_value[PITCH];
  roll_default = RX.channel_value[ROLL];

  // ------- Main loop -----------
  while(!abort)
  {
    // --------- Recieve -----------
    RX = DSM_UART.DSM_analyse(false, TX);

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
                             stop_value = TX.channel_value[6];
                             break;
            case REACT_HALFSPEED: state = STATE_HALFSPEED;
                                  break;
            default:         break; // No state change
          }
          break;
      case STATE_STOP:
          // _________ STOP STATE _________________
          // Keep constant throttle, everything else 0.
          TX = RX;

          if(RX.channel_value[PITCH] < pitch_default){
            TX.channel_value[PITCH] = pitch_default;
          }
          else{
            TX.channel_value[PITCH] =  ( (RX.channel_value[PITCH] - pitch_default) / 2 ) + pitch_default;
          }

          TX.channel_value[ROLL] =  ( (RX.channel_value[ROLL] - roll_default) / 2 ) + roll_default;
          TX.channel_value[YAW] =  ( (RX.channel_value[YAW] - yaw_default) / 2 ) + yaw_default;

          // Update state
          switch(local_reaction)
          {
            case REACT_FEEDBACK: state = STATE_FEEDBACK;
                                 break;
            case REACT_HALFSPEED: state = STATE_HALFSPEED;
                                  break;
            default:         break;
          }
          break;
       case STATE_HALFSPEED:
          // _________ HALFSPEED STATE ______________
          // Use half the speed
          TX = RX;

          TX.channel_value[PITCH] =  ( (RX.channel_value[PITCH] - pitch_default) / 2 ) + pitch_default;
          TX.channel_value[ROLL] =  ( (RX.channel_value[ROLL] - roll_default) / 2 ) + roll_default;
          TX.channel_value[YAW] =  ( (RX.channel_value[YAW] - yaw_default) / 2 ) + yaw_default;

          // Update state
          switch(local_reaction)
          {
            case REACT_FEEDBACK: state = STATE_FEEDBACK;
                                 break;
            case REACT_STOP:     state = STATE_HALFSPEED;
                                 stop_value = TX.channel_value[6];
                                 break;
            default:         break;
          }
          break;
       default: cout << "Error in state" << endl;
                abort = true;
                     break;
    }

    // Manual state changer
    // Mode 0: 1704 | Mode 1: 1192 | Mode 2: 340
    temp_FM = RX.channel_value[FLIGHT_MODE];

    if(temp_FM > 300 && temp_FM < 400)
      state = STATE_FEEDBACK;
    if(temp_FM > 1150 && temp_FM < 1250)
      state = STATE_HALFSPEED;
    if(temp_FM > 1650 && temp_FM < 1750)
      state = STATE_STOP;

    // Printing information
    system("clear"); // Clear terminal

    printf("---------- PACKETS ----------\n");
    printf("channel 0 | TX: %d \t RX: %d \n", TX.channel_value[0], RX.channel_value[0]);
    printf("channel 1 | TX: %d \t RX: %d \n", TX.channel_value[1], RX.channel_value[1]);
    printf("channel 2 | TX: %d \t RX: %d \n", TX.channel_value[2], RX.channel_value[2]);
    printf("channel 3 | TX: %d \t RX: %d \n", TX.channel_value[3], RX.channel_value[3]);
    printf("channel 4 | TX: %d \t RX: %d \n", TX.channel_value[4], RX.channel_value[4]);
    printf("channel 5 | TX: %d \t RX: %d \n", TX.channel_value[5], RX.channel_value[5]);
    printf("channel 6 | TX: %d \t RX: %d \n", TX.channel_value[6], RX.channel_value[6]);

    printf("\n ------ States And Reacts -------\n");
    printf(" State: %d \t React: %d \n", state, local_reaction);

    printf("\n ------ Default values -------\n");
    printf(" Pitch: %d \t Roll: %d \t Yaw: %d \t Throttle: %d \n", pitch_default, roll_default, yaw_default, throttle_default);

    if(RX.channel_value[PANIC_BIND] > 1000) // Stop at bind/panic button
      abort = true;
  }

  DSM_UART.DSM_analyse(true, RX);
  //pthread_join( CV_thread, NULL);

  return 0;
}
