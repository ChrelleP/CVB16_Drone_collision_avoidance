//----------------- Main file for test of serial communication -----------------

#include <cstdlib>
#include <pthread.h>
#include "stdio.h"
#include "serial_pi.hpp"
#include "wiringSerial.h"

using namespace std;

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

    return 0;
}
