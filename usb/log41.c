#define TERMINAL    "/dev/ttyACM2"

#include <errno.h>
#include <fcntl.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <signal.h>

static int keepRunning = 1;

void intHandler(int x) {
    keepRunning = 0;
}

/* Set *T to indicate raw mode.  */
void cfmakeraw (struct termios *t)
{
  t->c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL|IXON);
  t->c_oflag &= ~OPOST;
  t->c_lflag &= ~(ECHO|ECHONL|ICANON|ISIG|IEXTEN);
  t->c_cflag &= ~(CSIZE|PARENB);
  t->c_cflag |= CS8;
  t->c_cc[VMIN] = 1;		/* read returns when one char is available.  */
  t->c_cc[VTIME] = 0;
}

int set_interface_attribs(int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    cfmakeraw(&tty);
#if 0
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
#endif

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

int main(int argc, char *argv[])
{
    char *portname = TERMINAL;
    int fd;
    FILE *flog = fopen(argv[1],"wb");
    if (!flog) {
        printf("Error opening %s: %s\n", argv[1], strerror(errno));
        return -1;
    }

    fd = open(portname, O_RDWR | O_NOCTTY | O_NONBLOCK);
//    fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        printf("Error opening %s: %s\n", portname, strerror(errno));
        return -1;
    }
    /*baudrate 115200, 8 bits, no parity, 1 stop bit */
    //set_interface_attribs(fd, B115200);

    struct sigaction act;
    act.sa_handler = intHandler;
    sigaction(SIGINT, &act, NULL);

    /* simple noncanonical input */
    int done = 0;
    unsigned long len = 0;
    int nl=0;
    unsigned char buf[1024];
    int rdlen;
    while (keepRunning) {
        rdlen = read(fd, buf, sizeof(buf) - 1);
        if (rdlen > 0) {
            buf[rdlen] = 0;
            if( buf[0] == 0x0a && rdlen == 1 ) {
              if( nl++ == 0)
                continue;
            } else if( buf[0] == 'Q' ) {
              if( buf[1] == 'u' && buf[2] == 'i' ) {
                printf("Got stop signal!\nPress Ctrl+C to stop and save!\n");
                keepRunning=0;
                continue;
              }
            }
            fwrite(buf, 1, rdlen, flog);
            len += rdlen;
            //printf("[%s]\n", buf);
            printf("%ld\r", len);
            nl = 0;
        } else if (rdlen < 0) {
            //printf("Error from read: %d: %s\n", rdlen, strerror(errno));
        } else {  /* rdlen == 0 */
            printf("Timeout from read\n");
            keepRunning = 0;
        }               
        /* repeat read to get full message */
    }

    close(fd);
    fclose(flog);
    printf("\rAborted logging!\n%ld bytes written to <%s>\n", len, argv[1]);

    return done;
}
