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


class serial_pi
{
  public:

    serial_pi();

    void uart_tx();
    void uart_rx();

    ~serial_pi();

  private:

    int uart0_filestream_test;

    bool uart_setup();

};
