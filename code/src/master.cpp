////////////////////////////////////////////////////////////////////////////////
// Master file for bachelor project
// "Collision avoidance for UAVs in indoor Agriculture"
// An overview of the use and architecture can be found in the report.
//
// Made by: Christian Pedersen and Vincent Tofterup
////////////////////////////////////////////////////////////////////////////////

// ---------------------------- INCLUDES ---------------------------------------
#include <cstdlib>
#include <pthread.h>
#include <iostream>
#include <fstream>
#include "stdio.h"
#include "wiringSerial.h"
#include "wiringPi.h"
#include "feature_detection.hpp"
#include "object.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "DSM_analyser.hpp"
#include <unistd.h>

using namespace std;
using namespace cv;

// ----------------------------- DEFINES ---------------------------------------

#define STATE_STOP           1
#define STATE_ECHO           2
#define STATE_REDUCED        3
#define STATE_CHANGING       4

#define REACT_NOTHING        0
#define REACT_STOP           1
#define REACT_ECHO           2
#define REACT_REDUCED        3
#define REACT_LEFT           4
#define REACT_RIGHT          5

#define THROTTLE             0
#define YAW                  3      // Right -- | Left ++
#define PITCH                2      // Forward -- | Backwards ++
#define ROLL                 1      // Left -- | Right ++
#define FLIGHT_MODE          4      // Mode 0: 1704 | Mode 1: 1192 | Mode 2: 340
#define PANIC_BIND           5

#define LED		               21
#define STOP_DIST            150
#define REDUCED_DIST         400

// --------------------------- GLOBAL VARIABLES --------------------------------

volatile int global_reaction = REACT_NOTHING;     // Variables used for shared communication between threads
volatile float global_dist = 10000;
volatile int global_abort = false;

pthread_mutex_t thread_mutex = PTHREAD_MUTEX_INITIALIZER;     // Mutex (Semaphore) used to avoid conflicts between threads.

// --------------------------- FUNCTIONS ---------------------------------------
void *CV_avoid(void *arg)
{
   //------------- Create objects and variables -------------
   feature_detection FT;   // Feature Detection object - used for CV methods
   VideoCapture cap(0);       // Video Capture object - used to get frames from video
   VideoWriter fly_vid;

   int frame_width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
   int frame_height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);

   fly_vid.open( "/home/pi/CVB16/Data/last_flight.avi",
               CV_FOURCC('M','J','P','G'),
               10,
               Size(frame_width, frame_height) );

   //double FPS = cap.get(CV_CAP_PROP_FPS);
   //double WIDTH = cap.get(CV_CAP_PROP_FRAME_WIDTH);
   //double HEIGHT = cap.get(CV_CAP_PROP_FRAME_HEIGHT);

   //printf("FPS: %lf \t WIDTH: %lf \t HEIGHT: %lf \n", FPS, WIDTH, HEIGHT);

   bool CV_abort = false;

   //------------- Variables --------------------------------
   int local_reaction = REACT_NOTHING;
   float local_dist = 10000;
   int temp_reaction = REACT_NOTHING;
   float temp_dis = 10000;

   int LB_MASK = 60;                 // Lower bound for mask
   int UB_MASK = 100;                 // Upper bound for mask
   int AS_MEDIAN = 3;                  // Apperture size for median filter

   int LB_CANNY = 100;                // Lower bound for canny
   int UB_CANNY = 200;                // Upper bound for canny
   int AS_CANNY = 3;                  // Apperture size for canny filter

   float RHO = 0.8;                    // Rho used for HoughTransform
   float THETA = 0.75*(CV_PI/180);         // Theta used for HoughTransform
   int THRESHOLD = 100;              // Threshold for HoughTransform

   //------------------ While loop ---------------------------
   while(!CV_abort)
   {
     bool success = cap.read(FT.source);           // read a new frame from video

     if (!success)                                 //if not success, break loop
     {
       cout << "Could not read frame" << endl;
       break;
     }

     //printf("width: %i \t height: %i \n", FT.source.cols, FT.source.rows);

     FT.filter(LB_MASK, UB_MASK, AS_MEDIAN);         // Filter the image
     FT.edges(LB_CANNY, UB_CANNY, AS_CANNY);         // Find edges with canny
     FT.lines(RHO, THETA, THRESHOLD);                // Find lines through Hough

     FT.filter_houghlines();
     FT.identify_objects();

     pthread_mutex_lock( &thread_mutex );
     temp_reaction = global_reaction;
     pthread_mutex_unlock( &thread_mutex );

     local_dist = FT.calc_distance();
     local_reaction = FT.collision_risk(temp_reaction);

     pthread_mutex_lock( &thread_mutex );
     global_reaction = local_reaction;
     global_dist = local_dist;
     CV_abort = global_abort;
     pthread_mutex_unlock( &thread_mutex );

     //FT.draw_objects();
     //FT.draw_filtered_lines();
     //FT.draw_lines();
     //FT.show_source();
     //FT.show_filter();
     //FT.show_edge_map();

     fly_vid.write(FT.source);

     if(waitKey(1) == 27)                         // Wait 50 ms untill next frame, exit if escape is pressed
     {
       cout << "esc key is pressed by user" << endl;
       break;
     }
   }

   cap.release();
   fly_vid.release();

   pthread_exit(NULL);
}

int main ()
{
  // -------- Startup ---------
  DSM_RX_TX DSM_UART;

  ofstream flight_data;
  flight_data.open("/home/pi/CVB16/Data/flight_data.dat");

  // Syntax for data
  flight_data << "Distance;Reaction;";
  flight_data << "RX_Throttle;RX_Pitch;RX_Roll;RX_YAW;RX_Flight_Mode;";
  flight_data << "TX_Throttle;TX_Pitch;TX_Roll;TX_Yaw;TX_Flight_Mode" << endl;

  wiringPiSetup();
  pinMode(LED, OUTPUT);

  pthread_t CV_thread;
  int CV_rc;

  CV_rc = pthread_create( &CV_thread, NULL, CV_avoid, NULL);
  if( CV_rc )
    printf("Thread creation failed: %d\n", CV_rc);

  // ------ Variables -------------
  int state = STATE_ECHO;
  int local_reaction = REACT_NOTHING;
  float local_dist = 10000;
  int stop_value = 0;
  int packet_counter = 0;
  int temp_FM;
  int run = 0;
  int transition = 0;

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

  digitalWrite(LED, HIGH);

  // ------- Main loop -----------
  while(!abort)
  {
    // --------- Recieve -----------
    RX = DSM_UART.DSM_analyse(false, TX);

    // --------- State machine ----------
    // Retrieve reaction
    pthread_mutex_lock( &thread_mutex );
    local_reaction = global_reaction;
    local_dist = global_dist;
    pthread_mutex_unlock( &thread_mutex );

    // Switch on the state
    switch(state)
    {
      case STATE_ECHO:
          // _________ FEEDBACK STATE _____________
          TX = RX;

          //LED
          digitalWrite(LED, HIGH);

          // Update state
          switch(local_reaction)
          {
            case REACT_STOP: state = STATE_REDUCED;
                             break;
            case REACT_REDUCED: state = STATE_REDUCED;
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

          if(run > 0)
          {
            TX.channel_value[PITCH] = pitch_default + 200;
            run --;
          }

          // LED
          digitalWrite(LED, LOW);

          // Update state
          switch(local_reaction)
          {
            case REACT_ECHO:  if(transition == 0)
                              {
                                state = STATE_REDUCED;
                              }
                              else{
                                transition --;
                              }
                              break;
            case REACT_REDUCED: if(transition == 0)
                                {
                                  state = STATE_REDUCED;
                                }
                                else{
                                  transition --;
                                }
                                break;
            default:    transition = 30;
                        break;
          }
          break;
       case STATE_REDUCED:
          // _________ HALFSPEED STATE ______________
          // Use half the speed
          TX = RX;

          if(local_dist > 400)
      		{
      		    local_dist = 400;
      		}
      	  else if (local_dist < 150)
      		{
      		    local_dist = 150;
      		}

      	  TX.channel_value[PITCH] =  ( (RX.channel_value[PITCH] - pitch_default) * ( (0.002) * (local_dist - STOP_DIST) + 0.5 ) ) + pitch_default;
          TX.channel_value[ROLL] =  ( (RX.channel_value[ROLL] - roll_default) * ( (0.002) * (local_dist - STOP_DIST) + 0.5 ) ) + roll_default;
          TX.channel_value[YAW] =  ( (RX.channel_value[YAW] - yaw_default) * ( (0.002) * (local_dist - STOP_DIST) + 0.5 ) ) + yaw_default;

          if(RX.channel_value[PITCH] < pitch_default)
          {
            TX.channel_value[PITCH] = ( (RX.channel_value[PITCH] - pitch_default) * (0.004)*(local_dist - STOP_DIST) + pitch_default);
	  }

          // LED
          if(digitalRead(LED) == HIGH)
            digitalWrite(LED, LOW);
          else
            digitalWrite(LED, HIGH);

          // Update state
          switch(local_reaction)
          {
            case REACT_ECHO: if(transition == 0)
                              {
                                state = STATE_ECHO;
                              }
                              else{
                                transition --;
                              }
                              break;
            case REACT_STOP:     state = STATE_STOP;
                                 run = 23;
                                 break;
            default:    transition = 30;
                        break;
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
      state = STATE_ECHO;
    if(temp_FM > 1150 && temp_FM < 1250)
      state = STATE_REDUCED;
    if(temp_FM > 1650 && temp_FM < 1750)
      state = state;

    // Write data to file
    //printf("Calculated distance: %f\n",local_dist);
    flight_data << local_dist << ";" << local_reaction << ";";
    flight_data << RX.channel_value[THROTTLE] << ";" << RX.channel_value[PITCH] << ";" << RX.channel_value[ROLL] << ";" << RX.channel_value[YAW] << ";";
    flight_data << TX.channel_value[THROTTLE] << ";" << TX.channel_value[PITCH] << ";" << TX.channel_value[ROLL] << ";" << TX.channel_value[YAW] << endl;

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
    printf(" State: %d \t React: %d \t local_dist: %f \t run: %d \n", state, local_reaction, local_dist, run);

    printf("\n ------ Default values -------\n");
    printf(" Pitch: %d \t Roll: %d \t Yaw: %d \t Throttle: %d \n", pitch_default, roll_default, yaw_default, throttle_default);

    if(RX.channel_value[PANIC_BIND] > 1000) // Stop at bind/panic button
      abort = true;
  }

  pthread_mutex_lock( &thread_mutex );
  global_abort = true;
  pthread_mutex_unlock( &thread_mutex );

  flight_data.close();

  DSM_UART.DSM_analyse(true, RX);
  //pthread_join( CV_thread, NULL);

  return 0;
}
