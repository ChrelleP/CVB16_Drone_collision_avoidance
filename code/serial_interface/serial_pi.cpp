#include "serial_pi.hpp"

serial_pi::serial_pi()
{
  if( uart_setup() )
    printf("--UART setup done--");
  else
    printf("!! ERROR - Setup was unsuccesfull !!");
}

// THIS CODE IS INSPIRED BY THE INFORMATION FOUND ON: http://www.raspberry-projects.com/pi/programming-in-c/uart-serial-port/using-the-uart
bool serial_pi::uart_setup()
{
  // Generate a variable for the file descriptor
  int uart0_filestream = -1;


  // O_RDWR   - Open for reading and writing.
  // O_NOCTTY - When set and path identifies a terminal device,
  //            open() shall not cause the terminal device to become the controlling terminal for the process.
  // O_NDELAY - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
	//            if there is no input immediately available (instead of blocking). Likewise, write requests can also return
	//            immediately with a failure status if the output can't be written immediately.

  // Open device AMA0 - The UART on the Raspberry Pi
  // IMPORTANT: On the 3. generation Pi, the AMA0 is used for the bluetooth module - UART is now ...
  uart0_filestream = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);
	if (uart0_filestream == -1)
	{
		//Error message if serial port cannot open
    return false;
		printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
	}

  // Configuring the UART
  // Baudrate: 115200 - As set by the receiver
  // CSIZE: CS8 (CSIZE specifies the amount of bits pr. byte)
  // CLOCAL - Bit is set to ignore modem status lines (the carrier detect signal is ignored)
  // CREAD - Enable receiver (If this bit is set the input can be read from the terminal. Otherwise, input is discarded when it arrives.)
  struct termios options;
  	tcgetattr(uart0_filestream, &options);
  	options.c_cflag = B115200 | CS8 | CLOCAL | CREAD ;
  	options.c_iflag = IGNPAR;
  	options.c_oflag = 0;
  	options.c_lflag = 0;
  	tcflush(uart0_filestream, TCIFLUSH);
  	tcsetattr(uart0_filestream, TCSANOW, &options);

  return true;
}

serial_pi::~serial_pi()
{

}
