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

//number of amplifiers
int num_amp = 6;

int serialport_init (const char* serialport, int baud) {
    struct termios toptions;
    int fd;

    fd = open(serialport, O_RDWR | O_NONBLOCK);
    if (fd == -1) {
        perror("serialport_init: Unable to open port");
        return -1;
    }

    if (tcgetattr(fd, &toptions) < 0) {
        perror("serialport_init: Couldn't get term attributes");
        return -1;
    }

    speed_t brate = baud;
    switch (brate) {
        case 9600: brate = B9600; break;
    }
    cfsetispeed(&toptions, brate);
    cfsetospeed(&toptions, brate);

    // 8N1
    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    // no flow control
    toptions.c_cflag &= ~CRTSCTS;

    //toptions.c_cflag &= ~HUPCL; // disable hang-up-on-close to avoid reset

    toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    toptions.c_oflag &= ~OPOST; // make raw

    // see: http://unixwiz.net/techtips/termios-vmin-vtime.html
    toptions.c_cc[VMIN]  = 0;
    toptions.c_cc[VTIME] = 0;
    //toptions.c_cc[VTIME] = 20;

    tcsetattr(fd, TCSANOW, &toptions);
    if( tcsetattr(fd, TCSAFLUSH, &toptions) < 0) {
        perror("init_serialport: Couldn't set term attributes");
        return -1;
    }

    return fd;
}

int serialport_flus (int fd) {
    sleep(2);
    return tcflush(fd, TCIOFLUSH);
}

int serialport_writebyte (int fd, uint8_t b) {
    int n = write(fd, &b, 1);
    if (n!=1)
        return -1;
    return 0;
}

void error (char * msg) {
    fprintf(stderr, "%s\n", msg);
    exit(EXIT_FAILURE);
}

int serialport_read_until (int fd, char* buf, char until, int buf_max, int timeout) {
    char b[1];
    int i = 0;
    do {
        int n = read(fd, b, 1);
        if (n == -1) return -1;
        if (n == 0) {
            usleep(1*1000);
            timeout--;
            if (timeout == 0)   return -2;
            continue;
        }
        buf[i] = b[0];
        i++;
    } while ( b[0] != until && i < buf_max && timeout > 0 );

    buf[i] = 0;
    return 0;
}

int *init_amp(){

    int baudrate = 9600;

    *char **optarg = (char **)malloc(sizeof(int)*num_amp);

    for(int i = 0 ; i < num_amp ; i++)
    {
        optarg[i] = (char *)malloc(sizeof(char)*13);
    }

    sprintf(optarg[0], "/dev/ttyACM0");
    sprintf(optarg[1], "/dev/ttyACM1");
    sprintf(optarg[2], "/dev/ttyACM2");
    sprintf(optarg[3], "/dev/ttyACM3");
    sprintf(optarg[4], "/dev/ttyACM4");
    sprintf(optarg[5], "/dev/ttyACM5");

    int fd[num_amp] = {0};
    int rc[num_amp] = {0};

    for(int i = 0 ; i < num_amp ; i++)
    {
        fd[i] = serialport_init(optarg[i], baudrate);
        rc[i] = serialport_writebyte(fd[i], (uint8_t)131);
    }

    return fd;

}

int stop_amp(int fd[]){

    int rc[num_amp] = {0};

    //stops all the coils
    for(int j = 0 ; j < num_amp ; j++)
    {
        rc[j] = serialport_writebyte(fd[j], (uint8_t)224);//stop command
        //required before you can run the motor:
        rc[j] = serialport_writebyte(fd[j], (uint8_t)131);
    }

    return 0;
}

int run_amp(int fd[], double inpow[]){
    int baudrate = 9600;
    const int buf_max = 256;
    char eolchar = '\n';
    char buf[buf_max];
    int timeout = 1000;
    int speed, speed_byte_1, speed_byte_2;

    int rc[num_amp] = {0};

    //initialization for the motors
    for(int j = 0 ; j < num_amp ; j++)
    {
        rc[j] = serialport_writebyte(fd[j], (uint8_t)131);
    }

    //testing with speed (voltage)
    for(int j = 0 ; j < num_amp ; j++)
    {
        if(inpow[j] > 0 && inpow[j] < 75)
        {
            speed = (inpow[j]/100)*3200;
            speed_byte_1 = speed % 32;
            speed_byte_2 = speed / 32;
            rc[j] = serialport_writebyte(fd[j], (uint8_t)131);
            rc[j] = serialport_writebyte(fd[j], (uint8_t)133);
            rc[j] = serialport_writebyte(fd[j], (uint8_t)speed_byte_1);
            rc[j] = serialport_writebyte(fd[j], (uint8_t)speed_byte_2);
        }
        else if(inpow > -75)
        {
            speed = (inpow[j]/-100)*3200;
            speed_byte_1 = speed % 32;
            speed_byte_2 = speed / 32;
            rc[j] = serialport_writebyte(fd[j], (uint8_t)131);
            rc[j] = serialport_writebyte(fd[j], (uint8_t)134);
            rc[j] = serialport_writebyte(fd[j], (uint8_t)speed_byte_1);
            rc[j] = serialport_writebyte(fd[j], (uint8_t)speed_byte_2);
        }
        else
        {
            rc[j] = serialport_writebyte(fd[j], (uint8_t)131);
            rc[j] = serialport_writebyte(fd[j], (uint8_t)134);
            rc[j] = serialport_writebyte(fd[j], 0;
            rc[j] = serialport_writebyte(fd[j], 0;
        }
    }

    usleep(3e3);//sleep for 3 milliseconds

    return 0;
}
