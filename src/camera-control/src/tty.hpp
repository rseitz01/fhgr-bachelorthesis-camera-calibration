extern "C" {

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <termios.h>
#include <stdio.h>
#include <curses.h>

#include "err.h"

int set_interface_attribs (int fd, int speed, int parity);
int set_blocking (int fd, int should_block);

};

