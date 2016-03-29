// ----------------- Serial_pi.hpp ---------------------
// This class is used for receiving messages from a
// Spektrum satellite receiver and send them to a
// Autoquad flight controller through a raspberry pi
// running raspdebian.
//
// The port used is the standard UART port located at
// pin 8 and 10.
//
// Made by: Vincent and Christian - CVB16
// -----------------------------------------------------
#pragma once


#include <iostream>
#include <stdio.h>
#include <unistd.h>			//Used for UART
#include <fcntl.h>			//Used for UART
#include <termios.h>		//Used for UART
#include <sys/ioctl.h>  //Used for UART


class serial_pi
{
  public:

    serial_pi();

    int available_data();
    void uart_tx(const unsigned char x);
    void uart_rx();

    void close_UART();

    ~serial_pi();

  private:

    int uart0_filestream;

    bool uart_setup();
    void uart_flush();

};
