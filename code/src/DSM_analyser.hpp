/*****************************************************************************
* University of Southern Denmark
* UAS Center
* Mathias Højgaard Egeberg & Tobias Lundby
*
* MODULENAME.: DSM_analyser.hpp
* PROJECT....: Autonomous precision landing ground system
* DESCRIPTION: RX and TX of modified frames using the DSMX protocol
*
*****************************************************************************/

/***************************** Include files *******************************/
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdbool.h>

//#include <time.h>       /* time */
#include <sys/time.h>   /* time */

#include <wiringSerial.h>

/*****************************   Namespaces  *******************************/
using namespace std;

/*****************************   Structs   *******************************/
typedef struct package{
  int channel_value[16] = { };
  int byte_H[8];
  int byte_L[8];
} package;

/*****************************    Defines    *******************************/
#define BYTES_IN_FRAME      16
#define BAUD_RATE           115200
#define HIGH                1
#define LOW                 0
#define MAX_ERROR_BETWEEN_PACKETS 5
#define SYNC_BASE           0
#define SYNC_INCREMENT      45
#define SYNC_TOLERANCE      5
#define SAFE_ZONE_THRESHOLD 45 // Approximate number of packets ins 10 seconds (packets come with a frequency of ~45,5Hz)
#define RESET_SYNC_THRESHOLD 700
#define FATAL_SYNC_THRESHOLD 2000
#define CHANNEL0_DEFAULT    334
#define CHANNEL1_DEFAULT    1190
#define CHANNEL2_DEFAULT    1114
#define CHANNEL3_DEFAULT    1022
#define CHANNEL4_DEFAULT    1704
#define CHANNEL5_DEFAULT    340
#define CHANNEL6_DEFAULT    0
#define CHANNEL_MAXVALUE    1700

#define safe_mode_threshold 5 /* Amount of seconds before change from UNSAFE to SAFE mode */

#define DSM_S_IDLE          0 /* Used while there is no bytes to recieve and changes values if needed */
#define DSM_S_SAFE          1 /* Used if it has not made any errors in UNSAFE mode for safe_mode_threshold seconds */
#define DSM_S_UNSAFE        2 /* Used when it is not certain that the frames are synced correctly */
#define DSM_S_FATAL         3 /* Used when everything else fails */


/*****************************   Class   *******************************/
class DSM_RX_TX
{
public: // Methods
    DSM_RX_TX();
    DSM_RX_TX(char* port);

    package DSM_analyse(bool loop, package modified_package);
    void enable_all_max();
    void disable_all_max();
    void change_channel_offset(int channel, int offset);
    void change_channel_offsets(int channel0_offset_value, int channel1_offset_value, int channel2_offset_value, int channel3_offset_value, int channel4_offset_value, int channel5_offset_value, int channel6_offset_value);
    int get_in_channel_value(int channel);
    int get_out_channel_value(int channel);
    int get_packet_errors();
    int get_DSM_state();
    double get_time();
private: // Methods
    double currentTimeUs();
    void decode_channel_value(package &p, int byte);
    void set_channel_value(package &p,int channel, int value);
    void decode_packet(package &p_in);
    void change_packet_values(package &p_in, package &p_out);
    void RX_TX();
public: // Variables
private: // Variables
    bool debug_simple   = true;
    bool debug_medium   = false;
    bool debug_expert   = false;
    bool debug_insane   = false;
    bool debug_packet   = false;
    bool debug_errors   = true;
    bool debug_time     = false;
    bool debug_time_expert     = false;

    int DSM_STATE       = DSM_S_UNSAFE;
    bool safe_mode      = false; // Used when going from IDLE mode to either UNSAFE or SAFE
    bool fatal_error    = false;
    bool modify_packets = true;
    bool packet_modified = true;

    bool PREAMBLE       = true;
    bool BYTE_TYPE      = HIGH;
    bool in_sync        = true;
    int byte_in;
    int old_byte_in;
    int byte_counter    = 0;
    int last_sync_dist  = 0;
    int UNSAFE_counter  = 0;

    int success_bytes   = 0;

    int sync_value;
    int sync_value_expected         = SYNC_BASE;
    int sync_value_expected_next    = sync_value_expected + SYNC_INCREMENT;
    int safe_zone_syncs             = 0;

    double time_byte         = 0;
    double time_last_byte    = 0;
    const double frame_timeout = 0.01; // micro seconds
    double time_diff_current = 0;
    double time_diff_last = 0;

    int UNSAFE_syncs;
    int avail_bytes     = 0; // 0 since no avaliable bytes when starting up

    bool packet_max_value = false;

    int channel0_offset = 0;
    int channel1_offset = 0;
    int channel2_offset = 0;
    int channel3_offset = 0;
    int channel4_offset = 0;
    int channel5_offset = 0;
    int channel6_offset = 0;

    int packet_errors = 0;

    int ser_handle; // The serial connection (file descriptor)
    package package_in, package_out;
};

/*****************************   Methods / functions   *******************************/
DSM_RX_TX::DSM_RX_TX()
/*****************************************************************************
*   Input    :
*   Output   :
*   Function : Default constructor
******************************************************************************/
{
    printf("Program started\nTrying to open serial device...\n");


    if ((ser_handle = serialOpen("/dev/ttyAMA0", BAUD_RATE)) < 0)
    {
        fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
        while(true){;}
    }
    else
        printf("Serial device has been opened successfully\n");

    package_out.channel_value[0] = CHANNEL0_DEFAULT;
    package_out.channel_value[1] = CHANNEL1_DEFAULT;
    package_out.channel_value[2] = CHANNEL2_DEFAULT;
    package_out.channel_value[3] = CHANNEL3_DEFAULT;
    package_out.channel_value[4] = CHANNEL4_DEFAULT;
    package_out.channel_value[5] = CHANNEL5_DEFAULT;
    package_out.channel_value[6] = CHANNEL6_DEFAULT;

    DSM_STATE = DSM_S_IDLE; // Ensure startup in UNSAFE mode

    if (DSM_STATE == DSM_S_IDLE && safe_mode == false)
        printf("RX and TX ready to start in IDLE mode (safe_mode is false)\n");
}

DSM_RX_TX::DSM_RX_TX(char* port)
/*****************************************************************************
*   Input    : Port name formatted as a string.
*   Output   : None
*   Function : Overload constructor
******************************************************************************/
{
    printf("Program started\nTrying to open serial device...\n");

    if ((ser_handle = serialOpen(port, BAUD_RATE)) < 0)
    {
        fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
        while(true){;}
    }
    else
        printf("Serial device has been opened successfully\n");

    package_out.channel_value[0] = CHANNEL0_DEFAULT;
    package_out.channel_value[1] = CHANNEL1_DEFAULT;
    package_out.channel_value[2] = CHANNEL2_DEFAULT;
    package_out.channel_value[3] = CHANNEL3_DEFAULT;
    package_out.channel_value[4] = CHANNEL4_DEFAULT;
    package_out.channel_value[5] = CHANNEL5_DEFAULT;
    package_out.channel_value[6] = CHANNEL6_DEFAULT;

    DSM_STATE = DSM_S_IDLE; // Ensure startup in UNSAFE mode

    if (DSM_STATE == DSM_S_IDLE && safe_mode == false)
        printf("RX and TX ready to start in IDLE mode (safe_mode is false)\n");
}

double DSM_RX_TX::currentTimeUs()
/*****************************************************************************
*   Input    : None
*   Output   : Time in micro seconds
*   Function : Returns the current time in us
******************************************************************************/
{
    //timeval current;
    //gettimeofday(&current, 0);
    //struct timespec current;
    //clock_gettime(CLOCK_REALTIME, &current);
    //return (long long)current.tv_sec * 1000000000L + current.tv_nsec;
    //printf("time returned is %li \n", (long)current.tv_sec * 1000000L + current.tv_usec);
    //return (long)current.tv_sec * 1000000L + current.tv_usec;
    struct timeval tim;
    gettimeofday(&tim, NULL);
    if (debug_time_expert)
        printf("time returned is %f \n", (double)tim.tv_sec+(tim.tv_usec/1000000.0));
    return (double)tim.tv_sec+(tim.tv_usec/1000000.0);
}

void DSM_RX_TX::decode_channel_value(package &p,int byte)
/*****************************************************************************
*   Input    : Package, byte number
*   Output   : Changes the package channel content
*   Function : ??
******************************************************************************/
{
    int chan_num = (p.byte_H[byte+1]>>3) & 0x0f;
    p.channel_value[chan_num] = ((p.byte_H[byte+1] & 0x07)<<8) | p.byte_L[byte+1];
    if (debug_packet and chan_num == 5)
        printf("Channel %i=%i\n", chan_num, p.channel_value[chan_num]);
}

void DSM_RX_TX::set_channel_value(package &p, int channel, int value)
/*****************************************************************************
*   Input    : Package, channel number, and value
*   Output   : Changes the package channel content
*   Function : Sets the value of a channel
******************************************************************************/
{
   p.channel_value[channel] = value;
   p.byte_H[channel+1] = ((channel<<3)|((value & 0x700) >> 8));
   p.byte_L[channel+1] = (value & 0xFF);
}

void DSM_RX_TX::decode_packet(package &p_in)
/*****************************************************************************
*   Input    : 1 packet
*   Output   : Decoded values inside struct
*   Function : Decodes all the channel values
******************************************************************************/
{
    decode_channel_value(p_in, 0);
    decode_channel_value(p_in, 1);
    decode_channel_value(p_in, 2);
    decode_channel_value(p_in, 3);
    decode_channel_value(p_in, 4);
    decode_channel_value(p_in, 5);
    decode_channel_value(p_in, 6);
}

void DSM_RX_TX::change_packet_values(package &p_in, package &p_out)
/*****************************************************************************
*   Input    : 2 packages
*   Output   : Changes the package content
*   Function : Changes a packet to another
******************************************************************************/
{
    p_out = p_in;         // Echo package as standard
    if (packet_max_value) {
        set_channel_value(p_out,0,CHANNEL_MAXVALUE);
        set_channel_value(p_out,1,CHANNEL_MAXVALUE);
        set_channel_value(p_out,2,CHANNEL_MAXVALUE);
        set_channel_value(p_out,3,CHANNEL_MAXVALUE);
        set_channel_value(p_out,4,CHANNEL_MAXVALUE);
        set_channel_value(p_out,5,CHANNEL_MAXVALUE);
        set_channel_value(p_out,6,CHANNEL_MAXVALUE);
    } else {
        set_channel_value(p_out,0,p_in.channel_value[0]+channel0_offset);
        set_channel_value(p_out,1,p_in.channel_value[1]+channel1_offset);
        set_channel_value(p_out,2,p_in.channel_value[2]+channel2_offset);
        set_channel_value(p_out,3,p_in.channel_value[3]+channel3_offset);
        set_channel_value(p_out,4,p_in.channel_value[4]+channel4_offset);
        set_channel_value(p_out,5,p_in.channel_value[5]+channel5_offset);
        set_channel_value(p_out,6,p_in.channel_value[6]+channel6_offset);
    }
}

int DSM_RX_TX::get_in_channel_value(int channel)
/*****************************************************************************
*   Input    : Channel
*   Output   : Channel value
*   Function : Returns the value of the channel (RX frame)
******************************************************************************/
{
    return package_in.channel_value[channel];
}
int DSM_RX_TX::get_out_channel_value(int channel)
/*****************************************************************************
*   Input    : Channel
*   Output   : Channel value
*   Function : Returns the value of the channel (TX frame)
******************************************************************************/
{
    return package_out.channel_value[channel];
}

int DSM_RX_TX::get_packet_errors()
/*****************************************************************************
*   Input    : none
*   Output   : Packet errors
*   Function : Returns the number of packet errors
******************************************************************************/
{
    return packet_errors;
}

int DSM_RX_TX::get_DSM_state()
/*****************************************************************************
*   Input    : none
*   Output   : DSM state
*   Function : Returns the DSM state
******************************************************************************/
{
    return DSM_STATE;
}

double DSM_RX_TX::get_time()
/*****************************************************************************
*   Input    : none
*   Output   : time
*   Function : Returns the time
******************************************************************************/
{
    return currentTimeUs();
}

package DSM_RX_TX::DSM_analyse(bool loop, package modified_package)
/*****************************************************************************
*   Input    : If loop=true then it loops otherwise it runs once.
*   Output   : None
*   Function : Analyses the serial input
******************************************************************************/
{
  if (loop)
  {
      while(!fatal_error)
          RX_TX();

      printf("Encountered a FATAL ERROR - echoing serial bytes until termination\n");
      while (true)
          if(serialDataAvail(ser_handle))
              serialPutchar(ser_handle,serialGetchar(ser_handle));
  }
  else
  {
      change_packet_values(modified_package, package_in);
      while(DSM_STATE == DSM_S_IDLE) // During idle, the packet will be modified
        RX_TX();
      while(DSM_STATE != DSM_S_IDLE) // Recieve and transmit a packet
        RX_TX();
      //printf("Package received\n");
      return package_in;
  }
}

void DSM_RX_TX::RX_TX()
/*****************************************************************************
*   Input    : None
*   Output   : None
*   Function : Recieves and transmits bytes on the serial bus
******************************************************************************/
{
    switch (DSM_STATE) {
        case DSM_S_IDLE: // *** IDLE mode ***
            if (modify_packets and !packet_modified) {

                change_packet_values(package_in, package_out);
                packet_modified = true; // Do it once per packet
            } else if (!modify_packets)
                package_out = package_in;

            if(avail_bytes = serialDataAvail(ser_handle))
            {
                if (safe_mode)
                    DSM_STATE = DSM_S_SAFE;
                else
                    DSM_STATE = DSM_S_UNSAFE;
            }
            break;
        case DSM_S_SAFE: // *** SAFE mode ***
            if(avail_bytes = serialDataAvail(ser_handle))
            {
                // RX byte
                old_byte_in = byte_in;
                time_last_byte = time_byte;
                time_byte = currentTimeUs();
                time_diff_last = time_diff_current;
                time_diff_current = time_byte - time_last_byte;
                if (debug_time)
                    printf("Time between bytes are %f (calculated from %f and %f) and timeout is %f \n", time_byte - time_last_byte, time_byte, time_last_byte, frame_timeout);
                byte_in = serialGetchar(ser_handle); //RX byte
                byte_counter++;
                success_bytes++;

                if (debug_simple)
                    if (success_bytes%5000 == 0) {
                        printf("Recieved and transmitted %i bytes\n",success_bytes);
                    }

                // TX prevoius frame
                switch (BYTE_TYPE) {
                    case  HIGH:
                        serialPutchar(ser_handle,package_out.byte_H[ (byte_counter / 2) ]);
                        break;
                    case  LOW:
                        serialPutchar(ser_handle,package_out.byte_L[ (byte_counter / 2) -1]);
                        break;
                    default:
                        if (debug_expert)
                            printf("BYTE_TYPE bool has unrecognizable value\n");
                        DSM_STATE = DSM_S_FATAL;
                        printf("Encountered a FATAL ERROR - echoing serial bytes until termination\n");
                        fatal_error = true;
                        break;
                }

                // Interpret RX byte
                switch (PREAMBLE) {
                    case true: // *** PREAMBLE = true ***
                        switch (BYTE_TYPE) {
                            case  HIGH: // *** BYTE_TYPE = HIGH ***
                                package_in.byte_H[0] = byte_in;
                                BYTE_TYPE = LOW;

                                sync_value = (256*old_byte_in)+byte_in;
                                if (debug_insane) {
                                    printf("Value is: %i \n", sync_value);
                                    printf("Expected value: %i \n", sync_value == sync_value_expected);
                                    printf("Expected value but varying low: %i \n", sync_value >= sync_value_expected - MAX_ERROR_BETWEEN_PACKETS);
                                    printf("Expected value but varying high: %i \n", sync_value <= sync_value_expected + MAX_ERROR_BETWEEN_PACKETS);
                                    printf("Next value low: %i \n", (sync_value_expected_next - SYNC_TOLERANCE) < sync_value);
                                    printf("Next value high: %i \n", sync_value < (sync_value_expected_next + SYNC_TOLERANCE));
                                    printf("Expected value varying combined: %i \n", (sync_value >= sync_value_expected - MAX_ERROR_BETWEEN_PACKETS
                                            and sync_value <= sync_value_expected + MAX_ERROR_BETWEEN_PACKETS));
                                    printf("Expected value varying future combined: %i \n", (((sync_value_expected_next - SYNC_TOLERANCE) < sync_value)
                                        and (sync_value < (sync_value_expected_next + SYNC_TOLERANCE))));
                                    printf("Time last: %i \n", time_diff_last > frame_timeout);
                                    printf("Time: %i \n\n", time_diff_current > frame_timeout);
                                }
                                break; // Break for BYTE_TYPE HIGH
                            case  LOW: // *** BYTE_TYPE = LOW ***
                                package_in.byte_L[0] = byte_in;
                                BYTE_TYPE = HIGH;
                                sync_value = (256*old_byte_in)+byte_in;
                                //printf("******* Preamble ********* Sync_val: %i\n",sync_value);

                                if (debug_insane) {
                                    printf("Value is: %i \n", sync_value);
                                    printf("Expected value: %i \n", sync_value == sync_value_expected);
                                    printf("Expected value but varying low: %i \n", sync_value >= sync_value_expected - MAX_ERROR_BETWEEN_PACKETS);
                                    printf("Expected value but varying high: %i \n", sync_value <= sync_value_expected + MAX_ERROR_BETWEEN_PACKETS);
                                    printf("Next value low: %i \n", (sync_value_expected_next - SYNC_TOLERANCE) < sync_value);
                                    printf("Next value high: %i \n", sync_value < (sync_value_expected_next + SYNC_TOLERANCE));
                                    printf("Expected value varying combined: %i \n", (sync_value >= sync_value_expected - MAX_ERROR_BETWEEN_PACKETS
                                            and sync_value <= sync_value_expected + MAX_ERROR_BETWEEN_PACKETS));
                                    printf("Expected value varying future combined: %i \n", (((sync_value_expected_next - SYNC_TOLERANCE) < sync_value)
                                        and (sync_value < (sync_value_expected_next + SYNC_TOLERANCE))));
                                    printf("Time last: %i \n", time_diff_last > frame_timeout);
                                    printf("Time: %i \n\n", time_diff_current > frame_timeout);
                                }

                                if(((sync_value == sync_value_expected
                                    or (sync_value >= sync_value_expected - MAX_ERROR_BETWEEN_PACKETS
                                        and sync_value <= sync_value_expected + MAX_ERROR_BETWEEN_PACKETS))
                                    or (((sync_value_expected_next - SYNC_TOLERANCE) < sync_value)
                                        and (sync_value < (sync_value_expected_next + SYNC_TOLERANCE))))
                                    and (time_diff_last > frame_timeout))
                                {
                                    if ((sync_value >= sync_value_expected - MAX_ERROR_BETWEEN_PACKETS and sync_value <= sync_value_expected + MAX_ERROR_BETWEEN_PACKETS) and sync_value != sync_value_expected)
                                    {
                                        packet_errors += sync_value - sync_value_expected;
                                        if (debug_errors)
                                            printf("There have been %i errors\n",packet_errors);
                                    }
                                   sync_value_expected = sync_value;
                                   sync_value_expected_next = sync_value + SYNC_INCREMENT;
                                }
                                else
                                {
                                    if (debug_simple)
                                        printf("Switching to UNSAFE mode due to bad sync\n");
                                    safe_zone_syncs = 0;
                                    last_sync_dist = 0;
                                    safe_mode = false;
                                    UNSAFE_counter = 0;
                                    DSM_STATE = DSM_S_UNSAFE;
                                }

                                PREAMBLE = false;
                                break; // Break for BYTE_TYPE LOW
                            default: // *** BYTE_TYPE = not known ***
                                if (debug_expert)
                                    printf("BYTE_TYPE bool has unrecognizable value\n");
                                DSM_STATE = DSM_S_FATAL;
                                printf("Encountered a FATAL ERROR - echoing serial bytes until termination\n");
                                fatal_error = true; // This is really bad!
                                break; // Break for BYTE_TYPE not known
                        }
                        break; // Break for PREAMBLE true
                    case false:  // *** PREAMBLE = false ***
                        switch (BYTE_TYPE) {
                            case  HIGH: // *** BYTE_TYPE = HIGH ***
                                package_in.byte_H[ (byte_counter / 2) ] = byte_in;
                                BYTE_TYPE = LOW;
                                break; // Break for BYTE_TYPE HIGH
                            case  LOW: // *** BYTE_TYPE = LOW ***
                                package_in.byte_L[ (byte_counter / 2) - 1 ] = byte_in;
                                BYTE_TYPE = HIGH;
                                if (byte_counter >= BYTES_IN_FRAME) // If end of frame note this
                                {
                                    DSM_STATE = DSM_S_IDLE; // Complete packet has been recieved so go into idle
                                    packet_modified = false; // Indicate for IDLE mode that is has to modify the packet if nessecary
                                    safe_mode = true; // Indicate that it has to return to SAFE mode from IDLE mode when a byte is received
                                    PREAMBLE = true; // Next time a byte is received the first part is the preamble
                                    byte_counter = 0; // Next time a byte is received this is the 1 byte; note the increment at each receive
                                    decode_packet(package_in);
                                }
                                break; // Break for BYTE_TYPE LOW
                            default:
                                if (debug_expert)
                                    printf("BYTE_TYPE bool has unrecognizable value\n");
                                DSM_STATE = DSM_S_FATAL;
                                printf("Encountered a FATAL ERROR - echoing serial bytes until termination\n");
                                fatal_error = true;
                                break; // Break for BYTE_TYPE not known
                        }
                        break; // Break for PREAMBLE false
                    default:  // *** BYTE_TYPE = not known ***
                        if (debug_medium)
                            printf("PREAMBLE bool has unrecognizable value\n");
                        DSM_STATE = DSM_S_FATAL;
                        printf("Encountered a FATAL ERROR - echoing serial bytes until termination\n");
                        fatal_error = true; // This is really bad!
                        break; // Break for BYTE_TYPE not known
                }
            }
            break; // Break for DSM_STATE SAFE
        case DSM_S_UNSAFE: // *** UNSAFE mode ***
            if(avail_bytes = serialDataAvail(ser_handle))
            {
                UNSAFE_counter++;
                if (debug_expert and UNSAFE_counter%50 == 0) {
                    printf("%i bytes have been unsafe\n", UNSAFE_counter);
                    printf("%i is reset and %i is fatal\n", RESET_SYNC_THRESHOLD*16, FATAL_SYNC_THRESHOLD*16);
                }
                success_bytes = 0;
                old_byte_in = byte_in;
                time_last_byte = time_byte;
                time_byte = currentTimeUs();
                time_diff_last = time_diff_current;
                time_diff_current = time_byte - time_last_byte;
                if (debug_time)
                    printf("Time between bytes are %f (calculated from %f and %f) and timeout is %f \n", time_byte - time_last_byte, time_byte, time_last_byte, frame_timeout);
                byte_in = serialGetchar(ser_handle); //RX byte
                serialPutchar(ser_handle,byte_in); //TX byte

                sync_value = (256*old_byte_in)+byte_in;
                if (debug_insane and false) {
                    printf("Value is: %i \n", sync_value);
                    printf("Expected value: %i \n", sync_value == sync_value_expected);
                    printf("Expected value but varying low: %i \n", sync_value >= sync_value_expected - MAX_ERROR_BETWEEN_PACKETS);
                    printf("Expected value but varying high: %i \n", sync_value <= sync_value_expected + MAX_ERROR_BETWEEN_PACKETS);
                    printf("Next value low: %i \n", (sync_value_expected_next - SYNC_TOLERANCE) < sync_value);
                    printf("Next value high: %i \n", sync_value < (sync_value_expected_next + SYNC_TOLERANCE));
                    printf("Expected value varying combined: %i \n", (sync_value >= sync_value_expected - MAX_ERROR_BETWEEN_PACKETS
                            and sync_value <= sync_value_expected + MAX_ERROR_BETWEEN_PACKETS));
                    printf("Expected value varying future combined: %i \n", (((sync_value_expected_next - SYNC_TOLERANCE) < sync_value)
                        and (sync_value < (sync_value_expected_next + SYNC_TOLERANCE))));
                    printf("Time last: %i \n", time_diff_last > frame_timeout);
                    printf("Time: %i \n\n", time_diff_current > frame_timeout);
                }

                if(((sync_value == sync_value_expected
                    or (sync_value >= sync_value_expected - MAX_ERROR_BETWEEN_PACKETS
                        and sync_value <= sync_value_expected + MAX_ERROR_BETWEEN_PACKETS))
                    or (((sync_value_expected_next - SYNC_TOLERANCE) < sync_value)
                        and (sync_value < (sync_value_expected_next + SYNC_TOLERANCE))))
                    and (time_diff_last > frame_timeout))
                {
                    if ((sync_value >= sync_value_expected - MAX_ERROR_BETWEEN_PACKETS and sync_value <= sync_value_expected + MAX_ERROR_BETWEEN_PACKETS) and sync_value != sync_value_expected)
                    {
                        packet_errors += sync_value - sync_value_expected;
                        if (debug_errors)
                            printf("There have been %i errors\n",packet_errors);
                    }
                    //printf("Time last: %f \n", time_diff_last);
                    sync_value_expected = sync_value;
                    sync_value_expected_next = sync_value_expected + SYNC_INCREMENT;
                    if (debug_expert)
                        printf("Last_sync_dist: %i\n",last_sync_dist);
                    if((safe_zone_syncs > 0 && last_sync_dist == 15) || safe_zone_syncs == 0)
                    {
                        safe_zone_syncs++;
                        last_sync_dist = 0;
                        if (debug_expert)
                            printf("Safe_zone_zyncs: %i Sync_value: %i Sync_value_expected: %i Sync_value_expected_next: %i \n",safe_zone_syncs,sync_value,sync_value_expected,sync_value_expected_next);
                    }
                    else if(safe_zone_syncs>0)
                    {
                        safe_zone_syncs--;
                        last_sync_dist = 0;
                    }
                }
                else
                    last_sync_dist++;

                //DETERMINE WHEATHER OR NOT THE THING BETEETH IS VALID
                if(safe_zone_syncs >= SAFE_ZONE_THRESHOLD && last_sync_dist == 14)
                {
                    byte_counter = 0;
                    PREAMBLE = true;
                    BYTE_TYPE = HIGH;
                    DSM_STATE = DSM_S_SAFE;
                    if (debug_medium)
                        printf("Exiting unsafe zone\n");
                    break;
                }

                if (UNSAFE_counter == RESET_SYNC_THRESHOLD*16)
                {
                    if (debug_medium) {
                        printf("The expected sync value has been reset\n");
                    }
                    sync_value_expected = SYNC_BASE;
                    sync_value_expected_next = sync_value + SYNC_INCREMENT;
                }
                else if (UNSAFE_counter >= FATAL_SYNC_THRESHOLD*16)
                {
                    DSM_STATE = DSM_S_FATAL;
                    printf("Encountered a FATAL ERROR - echoing serial bytes until termination\n");
                    fatal_error = true;
                }
            }
            break; // Break for DSM_STATE UNSAFE
        case DSM_S_FATAL: // *** FATAL mode ***
            if(serialDataAvail(ser_handle))
                serialPutchar(ser_handle,serialGetchar(ser_handle));
            break;
        default:
            DSM_STATE = DSM_S_UNSAFE;
            break; // Break for DSM_STATE not known
    }
}

void DSM_RX_TX::enable_all_max()
/*****************************************************************************
*   Input    : None
*   Output   : None
*   Function : Sets all the packages to max value
******************************************************************************/
{
    packet_max_value = true;
    if (debug_simple)
        printf("All packages are now set to 1700");
}

void DSM_RX_TX::disable_all_max()
/*****************************************************************************
*   Input    : None
*   Output   : None
*   Function : Sets all the packages to max value
******************************************************************************/
{
    packet_max_value = false;
    if (debug_simple)
        printf("All packages are not set to 1700");
}

void DSM_RX_TX::change_channel_offsets(int channel0_offset_value, int channel1_offset_value, int channel2_offset_value, int channel3_offset_value, int channel4_offset_value, int channel5_offset_value, int channel6_offset_value)
/*****************************************************************************
*   Input    : None
*   Output   : None
*   Function : Sets the channel offsets
******************************************************************************/
{
    channel0_offset = channel0_offset_value;
    channel1_offset = channel1_offset_value;
    channel2_offset = channel2_offset_value;
    channel3_offset = channel3_offset_value;
    channel4_offset = channel4_offset_value;
    channel5_offset = channel5_offset_value;
    channel6_offset = channel6_offset_value;


    change_packet_values(package_in, package_out);
    packet_modified = true; // Do it once per packet


    if (packet_max_value)
        if (debug_simple)
            printf("Note that the offsets are overridden by max values, use disable_all_max to disable");
}

void DSM_RX_TX::change_channel_offset(int channel, int offset)
/*****************************************************************************
*   Input    : None
*   Output   : None
*   Function : Set the specific channel offset
******************************************************************************/
{
    switch (channel) {
        case 0:
            channel0_offset = offset;
            break;
        case 1:
            channel1_offset = offset;
            break;
        case 2:
            channel2_offset = offset;
            break;
        case 3:
            channel3_offset = offset;
            break;
        case 4:
            channel4_offset = offset;
            break;
        case 5:
            channel5_offset = offset;
            break;
        case 6:
            channel6_offset = offset;
            break;
        default:
            break;
    }

    /*
    change_packet_values(package_in, package_out);
    packet_modified = true; // Do it once per packet
    */

    if (packet_max_value)
        if (debug_simple)
            printf("Note that the offsets are overridden by max values, use disable_all_max to disable");
}
