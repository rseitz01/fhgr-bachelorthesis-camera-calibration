#include "tty.hpp"

extern "C" {


int set_interface_attribs (int fd, int speed, int parity)
{
    struct termios tty;
    if (tcgetattr (fd, &tty) != 0)
    {
        THROW("error %d from tcgetattr", errno);
    }

    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
                                                    // disable IGNBRK for mismatched speed tests; otherwise receive break
                                                    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
                                    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
        THROW ("error %d from tcsetattr", errno);
    }
    return 0;
error:
    return -1;
}

int set_blocking (int fd, int should_block)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        THROW("error %d from tggetattr", errno);
    }

    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    if (tcsetattr (fd, TCSANOW, &tty) != 0) {
        THROW("error %d setting term attributes", errno);
    }
    return 0;
error:
    return -1;
}


#if 0 //{{{
int main(void)
{
    char *portname = "/dev/ttyACM0";
    int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        THROW ("error %d opening %s: %s", errno, portname, strerror (errno));
    }

    set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
    set_blocking (fd, 0);                // set no blocking

    initscr();
    noecho();
    raw();
    //nodelay(stdscr, false);
    timeout(-1);

    char buf [4096] = {0};
    char cprev = 0;
    for(;;) {
        int c = getch();
        if(c != ERR) {
            if(c == 4) break; // ctrl+d
                              //printw("[%i]", c);
            if(c == 12) { // ctrl+l
                erase();
            } else {
                write (fd, &c, 1);
                //timeout(0);
            }
            usleep(5000);
        }

        int n = 0;
        //do {
        buf[0] = 0;
        n = read (fd, buf, sizeof(buf)/sizeof(*buf));
        //printw("[%i]", n);
        buf[n] = 0;
        for(int i = 0; i < n; ++i) {
            char c = buf[i];
            if(c == 127 || c == 12 || c == 3) {
                /* skip (3=ctrl+c, 12=ctrl+l) */
            } else if(c == '\n' && cprev == '\n') {
                /* skip */
            } else {
                printw("%c", c);
                refresh();
                //printw("[%i]%c", c, c);
            }
            cprev = c;
        }
        //} while(n > 0);
    }

error:
    endwin();

    return 0;

}

#endif //}}}

};

