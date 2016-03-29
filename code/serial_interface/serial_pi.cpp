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

serial_pi::uart_tx()
{
    //----- TX BYTES -----
  unsigned char tx_buffer[20];
  unsigned char *p_tx_buffer;

  p_tx_buffer = &tx_buffer[0];
  *p_tx_buffer++ = 'H';
  *p_tx_buffer++ = 'e';
  *p_tx_buffer++ = 'l';
  *p_tx_buffer++ = 'l';
  *p_tx_buffer++ = 'o';

  if (uart0_filestream != -1)
  {
    int count = write(uart0_filestream, &tx_buffer[0], (p_tx_buffer - &tx_buffer[0]));		//Filestream, bytes to write, number of bytes to write
    if (count < 0)
    {
      printf("UART TX error\n");
    }
  }
}

serial_pi::uart_rx()
{
  //----- CHECK FOR ANY RX BYTES -----
	if (uart0_filestream != -1)
	{
		// Read up to 255 characters from the port if they are there
		unsigned char rx_buffer[256];
		int rx_length = read(uart0_filestream, (void*)rx_buffer, 255);		//Filestream, buffer to store in, number of bytes to read (max)
		if (rx_length < 0)
		{
			//An error occured (will occur if there are no bytes)
		}
		else if (rx_length == 0)
		{
			//No data waiting
		}
		else
		{
			//Bytes received
			rx_buffer[rx_length] = '\0';
			printf("%i bytes read : %s\n", rx_length, rx_buffer);
		}
	}
}

serial_pi::~serial_pi()
{

}
