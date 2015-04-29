//
// Test is based on
// https://github.com/paparazzi/paparazzi/blob/master/sw/airborne/boards/ardrone/navdata.c
//
// compiled:
//   arm-linux-gnueabi-gcc -o navdata test_navdata.c
//


#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>   // for baud rates and options
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <assert.h>
#include <pthread.h>

#include <stdint.h>



int init()
{
  int navdata_fd = 0;

  // Check if the FD isn't already initialized
  if (navdata_fd <= 0) {
    navdata_fd = open("/dev/ttyO1", O_RDWR | O_NOCTTY); //O_NONBLOCK doesn't work

    if (navdata_fd < 0) {
      printf("[navdata] Unable to open navdata board connection(/dev/ttyO1)\n");
      //return FALSE;
    }

    // Update the settings of the UART connection
    fcntl(navdata_fd, F_SETFL, 0); //read calls are non blocking
    //set port options
    struct termios options;
    //Get the current options for the port
    tcgetattr(navdata_fd, &options);
    //Set the baud rates to 460800
    cfsetispeed(&options, B460800);
    cfsetospeed(&options, B460800);

    options.c_cflag |= (CLOCAL | CREAD); //Enable the receiver and set local mode
    options.c_iflag = 0; //clear input options
    options.c_lflag = 0; //clear local options
    options.c_oflag &= ~OPOST; //clear output options (raw output)

    //Set the new options for the port
    tcsetattr(navdata_fd, TCSANOW, &options);
  }

  // Stop acquisition
//  navdata_cmd_send(NAVDATA_CMD_STOP);
//  return TRUE;
  return navdata_fd;
}



/**
 * Read from fd even while being interrupted
 */
ssize_t full_read(int fd, uint8_t *buf, size_t count)
{
  /* Apologies for illiteracy, but we can't overload |read|.*/
  size_t readed = 0;

  while (readed < count) {
    ssize_t n = read(fd, buf + readed, count - readed);
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        continue;
      }
      return n;
    }
    readed += n;
  }
  return readed;
}



int main()
{
  uint8_t buf[256];

  int fd = init();
  if( fd == 0 )
  {
    fprintf( stderr, "ERROR opening device\n" );
    return -1;
  }
  int i;
  FILE *out = fopen("navdata.bin", "wb");
  for( i = 0; i < 100; i++ )
  {
    size_t num = full_read( fd, buf, 256 );
    fwrite( buf, sizeof(uint8_t), num, out );
  }
  fclose( out );
  return 0;
}

