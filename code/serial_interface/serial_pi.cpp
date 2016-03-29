#include "serial_pi.hpp"

serial_pi::serial_pi()
{
  if( uart_setup() )
    printf("--UART setup done--\n");
  else
    printf("!! ERROR - Setup was unsuccesfull !!\n");
}

// THIS CODE IS INSPIRED BY THE INFORMATION FOUND ON: http://www.raspberry-projects.com/pi/programming-in-c/uart-serial-port/using-the-uart
bool serial_pi::uart_setup()
{
  // Set filestream to -1, will be changed if everything opens correctly.
  uart0_filestream = -1;

  // O_RDWR   - Open for reading and writing.
  // O_NOCTTY - When set and path identifies a terminal device,
  //            open() shall not cause the terminal device to become the controlling terminal for the process.
  // O_NDELAY - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
	//            if there is no input immediately available (instead of blocking). Likewise, write requests can also return
	//            immediately with a failure status if the output can't be written immediately.

  // Open device AMA0 - The UART on the Raspberry Pi
  // IMPORTANT: On the 3. generation Pi, the AMA0 is used for the bluetooth module - UART is now ttyS0
  uart0_filestream = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);
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

void serial_pi::uart_flush()
{
  tcflush (uart0_filestream, TCIOFLUSH) ;
}

int serial_pi::available_data()
{
  int data;
  if (ioctl (uart0_filestream, FIONREAD, &data) == -1)
    return -1 ;
  return data ;
}

void serial_pi::uart_tx(const unsigned char x)
{
  if (uart0_filestream != -1)
  {
    int count = write(uart0_filestream, &x, 1);		//Filestream, bytes to write, number of bytes to write
    if (count < 0)
    {
      printf("UART TX error\n");
    }
  }
}

void serial_pi::uart_rx()
{
  unsigned char y;
  //----- CHECK FOR ANY RX BYTES -----
	if (uart0_filestream != -1)
	{
		int rx_length = read(uart0_filestream, &y, 1);		//Filestream, buffer to store in, number of bytes to read (max)
		if (rx_length < 0)
		{
      printf("Error encountered RX length < 0\n");
			//An error occured (will occur if there are no bytes)
		}
		else if (rx_length == 0)
		{
      printf("No data waiting - RX Length = 0\n");
			//No data waiting
		}
		else
		{
			//Bytes received
			printf("Char read : %c\n", y);
		}
	}
}

void serial_pi::close_UART()
{
  close(uart0_filestream);
}

serial_pi::~serial_pi()
{

}
