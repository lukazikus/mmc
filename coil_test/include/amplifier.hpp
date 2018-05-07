#ifndef AMPLIFIER
#define AMPLIFIER

#include "general_header.hpp"

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <string.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <stdlib.h>
#include <getopt.h>
#include <sys/time.h>
#include <math.h>

int serialport_init (const char* serialport, int baud);

int serialport_flus (int fd);

int serialport_writebyte (int fd, uint8_t b);

void error (char * msg);

int serialport_read_until (int fd, char* buf, char until, int buf_max, int timeout);

int *init_amp();    // init. amplifiers

int stop_amp ();

int run_amp(float inpow[]);

#endif
