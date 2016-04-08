//----------------- Main file for test of serial communication -----------------

#include <cstdlib>
#include <pthread.h>
#include "stdio.h"
#include "serial_pi.hpp"
#include "wiringSerial.h"

using namespace std;

struct data_packet{
  int throttle;
  int yaw;
  int pitch;
  int rotate;
};

volatile data_packet serial_data;
volatile bool feedback;

pthread_mutex_t data_mutex = PTHREAD_MUTEX_INITIALIZER;

void *UART(void *arg)
{

   int uart_fd;

   if((uart_fd = serialOpen ("/dev/ttyS0", 115200)) < 0)
   {
     cout << "Cannot open serial device" << endl;
     return 1 ;
   }

   while(true){

     while (serialDataAvail (uart_fd))
     {
       printf (" %3d", serialGetchar (fd)) ;
     }

   }

   /*serial_pi uart_fs;

   unsigned char transmit = 'a';
   uart_fs.uart_tx(transmit);

   while(uart_fs.available_data() == 0)
   {
     sleep(1);
   }

   uart_fs.uart_rx();*/

   pthread_mutex_lock( &data_mutex );
   printf("\nThrottle: %d\n", serial_data.throttle);
   printf("Yaw: %d\n", serial_data.yaw);
   printf("Pitch: %d\n", serial_data.pitch);
   printf("Rotate: %d\n", serial_data.rotate);
   pthread_mutex_unlock( &data_mutex );

   pthread_exit(NULL);
}

int main ()
{
    printf("--------------- Serial Communication ---------------\n");

    int fd ;

    if ((fd = serialOpen ("/dev/ttyAMA0", 115200)) < 0)
    {
      fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
      return 1 ;
    }

    if (wiringPiSetup () == -1)
    {
      fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
      return 1 ;
    }

    while(true)
    {
      while (serialDataAvail (fd))
      {
        printf ("char: %3d \n", serialGetchar (fd));
      }
    }

    /*
    pthread_mutex_lock( &data_mutex );
    serial_data.throttle = 100;
    serial_data.yaw = 200;
    serial_data.pitch = 300;
    serial_data.rotate = 400;
    pthread_mutex_unlock( &data_mutex );

    pthread_t uart_thread;
    int uart_rc;

    uart_rc = pthread_create( &uart_thread, NULL, UART, NULL);
    if( uart_rc )
      printf("Thread creation failed: %d\n", uart_rc);

    pthread_join( uart_thread, NULL);
    */

    return 0;
}
