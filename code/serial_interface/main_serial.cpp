//----------------- Main file for test of serial communication -----------------

#include <cstdlib>
#include <pthread.h>
#include "stdio.h"
#include "serial_pi.hpp"

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

   serial_pi uart_fs;

   pthread_mutex_lock( &data_mutex );
   printf("Throttle: %d\n", serial_data.throttle);
   printf("Yaw: %d\n", serial_data.yaw);
   printf("Pitch: %d\n", serial_data.pitch);
   printf("Rotate: %d\n", serial_data.rotate);
   pthread_mutex_unlock( &data_mutex );

   pthread_exit(NULL);
}

int main ()
{
    printf("--------------- Serial Communication ---------------\n");

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

    return 0;
}
