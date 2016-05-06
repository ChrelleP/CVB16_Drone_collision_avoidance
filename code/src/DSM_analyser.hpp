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

#include <time.h>       /* time */
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
#define SYNC_TOLERANCE      5
#define SAFE_ZONE_THRESHOLD 50 // Approximate number of packets ins 10 seconds (packets come with a frequency of ~45,5Hz)
#define RESET_SYNC_THRESHOLD 700
#define FATAL_SYNC_THRESHOLD 2000
#define CHANNEL0_DEFAULT    376
#define CHANNEL1_DEFAULT    1016
#define CHANNEL2_DEFAULT    920
#define CHANNEL3_DEFAULT    1006
#define CHANNEL4_DEFAULT    340
#define CHANNEL5_DEFAULT    340
#define CHANNEL6_DEFAULT    0
#define CHANNEL_MAXVALUE    1700

#define safe_mode_threshold 10 /* Amount of seconds before change from UNSAFE to SAFE mode */

#define DSM_S_IDLE          0 /* Used while there is no bytes to recieve and changes values if needed */
#define DSM_S_SAFE          1 /* Used if it has not made any errors in UNSAFE mode for safe_mode_threshold seconds */
#define DSM_S_UNSAFE        2 /* Used when it is not certain that the frames are synced correctly */

/*****************************   Class   *******************************/
class DSM_RX_TX
{
public: // Methods
    DSM_RX_TX();
    DSM_RX_TX(char* port);
    package DSM_analyse(bool loop, package modified_package);
    void enable_all_max();
    void disable_all_max();
    void change_channel_offsets(int channel1, int channel2, int channel3, int channel4, int channel5, int channel6);
private: // Methods
    long long currentTimeUs();
    void set_channel_value(package &p,int channel, int value);
    void decode_channel_value(package &p, int byte);
    void decode_packet(package &p_in);
    void change_packet_values(package &p_in, package &p_out);
    void RX_TX();
private: // Variables
    int DSM_STATE = DSM_S_UNSAFE;

    int ser_handle; // The serial connection (file descriptor)
    package package_in, package_out; // Packages for transmitting and receiving.

    bool safe_mode = false; // Used when going from IDLE mode to either UNSAFE or SAFE
    bool fatal_error = false;
    bool modify_packets = true;
    bool packet_modified = true;

    bool PREAMBLE = true;
    bool BYTE_TYPE = HIGH;
    bool in_sync = true;
    int byte_in;
    int old_byte_in;
    int byte_counter = 0;
    int last_sync_dist = 0;
    int UNSAFE_counter = 0;

    int sync_value;
    int sync_value_expected = 0;
    int sync_value_expected_next = sync_value_expected + 45;
    int safe_zone_syncs = 0;

    long long time_byte = 0;
    long long time_last_byte = 0;
    const int frame_timeout = 5000; // micro seconds 1ms=1000us

    int UNSAFE_syncs;
    int avail_bytes = 0; // 0 since no avaliable bytes when starting up

    bool packet_max_value = false;

    int channel1_offset = 0;
    int channel2_offset = 0;
    int channel3_offset = 0;
    int channel4_offset = 0;
    int channel5_offset = 0;
    int channel6_offset = 0;
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

    if (DSM_STATE = DSM_S_IDLE && safe_mode == false)
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

    if (DSM_STATE = DSM_S_IDLE && safe_mode == false)
        printf("RX and TX ready to start in IDLE mode (safe_mode is false)\n");
}

long long DSM_RX_TX::currentTimeUs()
/*****************************************************************************
*   Input    : None
*   Output   : Time in micro seconds
*   Function : Returns the current time in us
******************************************************************************/
{
    timeval current;
    gettimeofday(&current, 0);
    return (long long)current.tv_sec * 1000000L + current.tv_usec;
}

void DSM_RX_TX::set_channel_value(package &p,int channel, int value)
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

void DSM_RX_TX::decode_channel_value(package &p,int byte)
/*****************************************************************************
*   Input    : Package, byte number
*   Output   : Changes the package channel content
*   Function : ??
******************************************************************************/
{
    int chan_num = (p.byte_H[byte+1]>>3) & 0x0f;
    p.channel_value[chan_num] = ((p.byte_H[byte+1] & 0x07)<<8) | p.byte_L[byte+1];
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
    //p_out = p_in;         // Echo package as standard
    if (packet_max_value) {
        set_channel_value(p_out,0,CHANNEL_MAXVALUE);
        set_channel_value(p_out,1,CHANNEL_MAXVALUE);
        set_channel_value(p_out,2,CHANNEL_MAXVALUE);
        set_channel_value(p_out,3,CHANNEL_MAXVALUE);
        set_channel_value(p_out,4,CHANNEL_MAXVALUE);
        set_channel_value(p_out,5,CHANNEL_MAXVALUE);
        set_channel_value(p_out,6,CHANNEL_MAXVALUE);
    } else {
        set_channel_value(p_out,0,p_in.channel_value[0]+channel1_offset);
        set_channel_value(p_out,1,p_in.channel_value[1]+channel1_offset);
        set_channel_value(p_out,2,p_in.channel_value[2]+channel2_offset);
        set_channel_value(p_out,3,p_in.channel_value[3]+channel3_offset);
        set_channel_value(p_out,4,p_in.channel_value[4]+channel4_offset);
        set_channel_value(p_out,5,p_in.channel_value[5]+channel5_offset);
        set_channel_value(p_out,6,p_in.channel_value[6]+channel6_offset);
    }
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
                byte_in = serialGetchar(ser_handle); //RX byte
                byte_counter++;

                // TX prevoius frame
                switch (BYTE_TYPE) {
                    case  HIGH:
                        serialPutchar(ser_handle,package_out.byte_H[ (byte_counter / 2) ]);
                        break;
                    case  LOW:
                        serialPutchar(ser_handle,package_out.byte_L[ (byte_counter / 2) -1]);
                        break;
                    default:
                        printf("BYTE_TYPE bool has unrecognizable value\n");
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
                                break; // Break for BYTE_TYPE HIGH
                            case  LOW: // *** BYTE_TYPE = LOW ***
                                package_in.byte_L[0] = byte_in;
                                BYTE_TYPE = HIGH;
                                sync_value = (256*old_byte_in)+byte_in;
                                //printf("******* Preamble ********* Sync_val: %i\n",sync_value);

                                if(sync_value == sync_value_expected
                                    || (((sync_value_expected_next - SYNC_TOLERANCE) < sync_value)
                                    && (sync_value < (sync_value_expected_next + SYNC_TOLERANCE))))
                                {
                                   sync_value_expected = sync_value;
                                   sync_value_expected_next = sync_value + 45;
                                }
                                else
                                {
                                    printf("Switching to UNSAFE mode due to bad sync\n");
                                    safe_zone_syncs = 0;
                                    last_sync_dist = 0;
                                    //safe_mode = false;
                                    UNSAFE_counter = 0;
                                    //DSM_STATE = DSM_S_UNSAFE;
                                }

                                PREAMBLE = false;
                                break; // Break for BYTE_TYPE LOW
                            default: // *** BYTE_TYPE = not known ***
                                printf("BYTE_TYPE bool has unrecognizable value\n");
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
                                    decode_packet(package_in); // Change channel values into the new values
                                    DSM_STATE = DSM_S_IDLE; // Complete packet has been recieved so go into idle
                                    packet_modified = false; // Indicate for IDLE mode that is has to modify the packet if nessecary
                                    safe_mode = true; // Indicate that it has to return to SAFE mode from IDLE mode when a byte is received
                                    PREAMBLE = true; // Next time a byte is received the first part is the preamble
                                    byte_counter = 0; // Next time a byte is received this is the 1 byte; note the increment at each receive
                                }
                                break; // Break for BYTE_TYPE LOW
                            default:
                                printf("BYTE_TYPE bool has unrecognizable value\n");
                                fatal_error = true;
                                break; // Break for BYTE_TYPE not known
                        }
                        break; // Break for PREAMBLE false
                    default:  // *** BYTE_TYPE = not known ***
                        printf("PREAMBLE bool has unrecognizable value\n");
                        fatal_error = true; // This is really bad!
                        break; // Break for BYTE_TYPE not known
                }
            }
            break; // Break for DSM_STATE SAFE
        case DSM_S_UNSAFE: // *** UNSAFE mode ***
            if(avail_bytes = serialDataAvail(ser_handle))
            {
                UNSAFE_counter++;
                old_byte_in = byte_in;
                time_last_byte = time_byte;
                time_byte = currentTimeUs();
                byte_in = serialGetchar(ser_handle); //RX byte
                serialPutchar(ser_handle,byte_in); //TX byte

                sync_value = (256*old_byte_in)+byte_in;
                if(sync_value == sync_value_expected ||
                    (((sync_value_expected_next - SYNC_TOLERANCE) < sync_value) &&
                       (sync_value < (sync_value_expected_next + SYNC_TOLERANCE))) &&
                    ((time_byte - time_last_byte) > frame_timeout))
                {
                    sync_value_expected = sync_value;
                    sync_value_expected_next = sync_value + 45;
                    printf("Last_sync_dist: %i\n",last_sync_dist);
                    if((safe_zone_syncs > 0 && last_sync_dist == 15) || safe_zone_syncs == 0)
                    {
                        safe_zone_syncs++;
                        last_sync_dist = 0;
                        printf("Safe_zone_zyncs: %i Sync_value: %i Sync_value_expected: %i Sync_value_expected_next: %i \n",safe_zone_syncs,sync_value,sync_value_expected,sync_value_expected_next);
                    }
                    else if(safe_zone_syncs>0)
                        safe_zone_syncs--;
                }
                else
                    last_sync_dist++;

                //DETERMINE WHEATHER OR NOT THE THING BETEETH IS VALID
                if(safe_zone_syncs == SAFE_ZONE_THRESHOLD && last_sync_dist == 14)
                {
                    byte_counter = 0;
                    PREAMBLE = true;
                    BYTE_TYPE = HIGH;
                    DSM_STATE = DSM_S_SAFE;
                    printf("Exiting unsafe zone\n");
                    break;
                }

                if (UNSAFE_counter == RESET_SYNC_THRESHOLD*16)
                {
                    sync_value_expected = 0;
                    sync_value_expected_next = sync_value + 45;
                }
                else if (UNSAFE_counter >= FATAL_SYNC_THRESHOLD*16)
                {
                    fatal_error = true;
                }
            }
            break; // Break for DSM_STATE UNSAFE
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
    printf("All packages are not set to 1700");
}

void DSM_RX_TX::change_channel_offsets(int channel1, int channel2, int channel3, int channel4, int channel5, int channel6)
/*****************************************************************************
*   Input    : None
*   Output   : None
*   Function : Sets the channel offsets
******************************************************************************/
{
    channel1_offset = channel1;
    channel2_offset = channel2;
    channel3_offset = channel3;
    channel4_offset = channel4;
    channel5_offset = channel5;
    channel6_offset = channel6;

    if (packet_max_value)
        printf("Note that the offsets are overridden by max values, use disable_all_max to disable");
}
