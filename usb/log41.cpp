#define TERMINAL    "/dev/ttyACM2"

#include <errno.h>
#include <fcntl.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <signal.h>

#include <map>

static int keepRunning = 1;

typedef struct label
{
    int   address;
    char  lbl[7];
} label_t;

using namespace std;

map<int, char *> mLabels;
map<int, char *>::iterator iLbls;


void readSymbols(const char *symFile)
{
  char sBuf[1024];
  char *p;
  int addr;
  char lbl[10];
  printf("Try to read symbols from \"%s\".\n", symFile);
  FILE *fp = fopen(symFile, "r");
  if( !fp ) {
    printf("Can't open file <%s>!\n", symFile);
  }
  while( fgets(sBuf, 1024, fp) ) {
    if( sscanf(sBuf, "%4X     %6s", &addr, lbl) ) {
      mLabels[addr] = strdup(lbl);
      //printf("%6.6s @ %04X\n", lbl, addr);
    }
  }

#if 0
   for(iLbls=mLabels.begin(); iLbls!=mLabels.end(); ++iLbls){
      printf("%s --> %d\n", iLbls->second, iLbls->first);
   }
   printf("Test: %s\n", mLabels[0x27F8]);
#endif
}

/**
 * Program exit codes
 */
enum exit_codes {
  EC_OK = 0, EC_ARG, EC_SIGNAL, EC_FATAL, EC_IO, EC_END
};

void eexit(int code, const char* err_msg, ...);

/**
 * Signal handler, exits with the error code arg signum
 *
 * @param int signum
 * @return void
 */
void signal_handler(int signum)
{
  printf("signal_handler(%d)\n", signum);
  int code = EC_SIGNAL;
  const char *signal_s;
  switch(signum) {
    case SIGTERM: signal_s = "kill signal"; break;
    case SIGKILL: signal_s = "hard kill signal"; break;
    case SIGINT: signal_s = "CTRL-C"; code=0; keepRunning = 0; break;
    case SIGHUP: signal_s = "hangup signal (SIGHUP)"; break;
    case SIGABRT: signal_s = "error in program (SIGABRT)"; break;
    case SIGTRAP: signal_s = "debug trap (TRAP)"; break;
    case SIGPIPE: signal_s = "broken pipe"; break;
    case SIGSYS: signal_s = "Program error (bad system call)"; break;
    default: signal_s = "(unknown signal)";
  }
  eexit(code, "Exit due to signal: %s", signal_s);
}

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

int fd;
FILE *flog = NULL;

int main(int argc, char *argv[])
{

    const char *portname = TERMINAL;
    flog = fopen(argv[1],"wb");
    if (!flog) {
        printf("Error opening %s: %s\n", argv[1], strerror(errno));
        return -1;
    }

    fd = open(portname, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        printf("Error opening %s: %s\n", portname, strerror(errno));
        return -1;
    }

    readSymbols("SYSTEMLABELS.TXT");

  //  struct sigaction act;
  ///  act.sa_handler = intHandler;
//    sigaction(SIGINT, &act, NULL);

  printf("main() Setup signals\n");
  // Signals
  signal(SIGTERM, signal_handler);
  signal(SIGKILL, signal_handler);
  signal(SIGINT,  signal_handler);
  signal(SIGHUP,  signal_handler);
  signal(SIGABRT, signal_handler);
  signal(SIGTRAP, signal_handler);
  signal(SIGPIPE, signal_handler);
  signal(SIGSYS,  signal_handler);

    /* simple noncanonical input */
    int done = 0;
    unsigned long len = 0;
    int nl=0;
    unsigned char buf[1024];
    int rdlen, addr;
    char *p;
    char *sym;
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
            // Does the line contain a symbol ([xxxx])?
            p = strchr((char*)buf, '[');
            if( p && *(p+5) == ']' && *(p+6) == 0xa ) {
              if( sscanf(p, "[%4X]", &addr) == 1 ) {
                sym = mLabels[addr];
                if( sym ) {
                  sprintf(p, "%s [%04X]\n", sym, addr);
                  rdlen = strlen((char*)buf);
                }
              }
            }
            // Check for other labels ...
            if( (p=strstr((char*)buf, "C XQ ")) || (p=strstr((char*)buf, "C GO ")) ) {
              if( sscanf(p+5, "%4X", &addr) == 1 ) {
                sym = mLabels[addr];
                if( sym ) {
                  sprintf(p+5, "%s [%04X]\n", sym, addr);
                  rdlen = strlen((char*)buf);
                }
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

/**
 * Prints an error message and exits.
 *
 * @param err_msg
 * @param ...
 * @return void
 */
void eexit(int code, const char* err_msg, ...)
{

//    close(fd);
    fclose(flog);
    printf("\rAborted logging!\n");
/*
  int i;
  //debug("eexit(%d, %s)", code, err_msg?err_msg:"(no message)");
 
  // Close and restore
  serial_port_close(&port);
 
  // Print last lines ...
  set_color(code!=0 ? COLOR_ERR : COLOR_OK, 1);
  if(is_verbose || code != 0) {
    if(!err_msg) {
      fprintf(stderr, "\n[Unexpected error]");
    } else {
      char buffer[256];
      va_list args;
      va_start (args, err_msg);
      vsnprintf (buffer, sizeof(buffer), err_msg, args);
      va_end (args);
      buffer[sizeof(buffer)-1] = '\0';
      if(err_msg) fprintf(stderr, "\n[%s]", buffer);
    }
    if(is_verbose) {
      fprintf(stderr, "[E%d]\n[TX:%llu, RX:%llu, RX:%llu lines]", code, bytes_tx,
          bytes_rx, lines_received);
    }
    fprintf(stderr, "\n");
  }
  debug("eexit() EXIT");
  set_color(COLOR_NO, 0);
  set_color(COLOR_NO, 1);
 
  if(in_d >= 0) {
    for(i=0; i<sizeof(stdios_orig); i++) {
      if(((unsigned char*)&stdios_orig)[i]) {
        tcsetattr(in_d, TCSANOW, &stdios_orig);
        break;
      }
    }
    close(in_d);
    in_d = -1;
  }
  if(out_d >= 0) {
    close(out_d);
    out_d = -1;
  }
 */
  exit(code);
}