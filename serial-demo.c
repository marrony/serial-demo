#include <errno.h>
#include <fcntl.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

int set_interface_attribs(int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

void set_mincount(int fd, int mcount)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error tcgetattr: %s\n", strerror(errno));
        return;
    }

    tty.c_cc[VMIN] = mcount ? 1 : 0;
    tty.c_cc[VTIME] = 5;        /* half second timer */

    if (tcsetattr(fd, TCSANOW, &tty) < 0)
        printf("Error tcsetattr: %s\n", strerror(errno));
}


int main(int argc, char* argv[]) {
    char *portname = argv[1];
    int fd;

    fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
      printf("Error opening %s: %s\n", portname, strerror(errno));
      return -1;
    }
    /*baudrate 115200, 8 bits, no parity, 1 stop bit */
    set_interface_attribs(fd, B9600);
    //set_mincount(fd, 0);                /* set to pure timed read */

    //fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL) | O_NONBLOCK);

    /* simple noncanonical input */
    do {
      unsigned char buf[80];
      int rdlen, wlen;

      printf("Input\n");

      rdlen = 0;
      while (rdlen < 80 && read(STDIN_FILENO, buf + rdlen, 1) > 0) {
        if (buf[rdlen] == '\r' || buf[rdlen] == '\n') {
          buf[rdlen] = 0;
          break;
        }
        rdlen++;
	printf("Len %d\n", rdlen);
      }

      printf("Input: %s %d %d\n", buf, rdlen, strlen(buf));
      if (rdlen > 0) {
        wlen = write(fd, buf, rdlen);
        if (wlen != rdlen) {
          printf("Error from write: %d, %d\n", wlen, errno);
        }
        //tcdrain(fd);    /* delay for output */

        while ((rdlen = read(fd, buf, 1)) == 1) {
          buf[1] = 0;
          printf("Read: \"%s\"\n", buf);

	  if (buf[0] == '#')
            break;
	}
      }
    } while (1);
}

