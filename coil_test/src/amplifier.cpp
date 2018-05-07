#include "amplifier.hpp"

/* file-wide variable */
const int num_amp = 6;                          //number of amplifiers
static int fd[num_amp] = {0,0,0,0,0,0};
int rc[num_amp] = {0,0,0,0,0,0};

int serialport_init (const char* serialport, int baud) {
    struct termios toptions;

    int fd1 = open(serialport, O_RDWR | O_NONBLOCK);
    if (fd1 == -1) {
        perror("serialport_init: Unable to open port");
        return -1;
    }

    if (tcgetattr(fd1, &toptions) < 0) {
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

    tcsetattr(fd1, TCSANOW, &toptions);
    if( tcsetattr(fd1, TCSAFLUSH, &toptions) < 0) {
        perror("init_serialport: Couldn't set term attributes");
        return -1;
    }

    return fd1;
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

/* init. amplifiers */
int * init_amp(){
    int baudrate = 9600;

    //fd[0] = serialport_init("/dev/ttyACM0", baudrate);
    fd[0] = serialport_init("/dev/serial/by-id/usb-Pololu_Corporation_Pololu_Simple_Motor_Controller_18v7_52FF-6E06-7283-5255-2916-2567-if00", baudrate);
    rc[0] = serialport_writebyte(fd[0], (uint8_t)131);

    //fd[1] = serialport_init("/dev/ttyACM1", baudrate);
    fd[1] = serialport_init("/dev/serial/by-id/usb-Pololu_Corporation_Pololu_Simple_Motor_Controller_18v7_49FF-7206-7277-5052-2531-1367-if00", baudrate);
    rc[1] = serialport_writebyte(fd[1], (uint8_t)131);

    //fd[2] = serialport_init("/dev/ttyACM2", baudrate);
    fd[2] = serialport_init("/dev/serial/by-id/usb-Pololu_Corporation_Pololu_Simple_High-Power_Motor_Controller_18v15_49FF-6D06-7277-5052-3251-0967-if00", baudrate);
    rc[2] = serialport_writebyte(fd[2], (uint8_t)131);

    //fd[3] = serialport_init("/dev/ttyACM3", baudrate);
    fd[3] = serialport_init("/dev/serial/by-id/usb-Pololu_Corporation_Pololu_Simple_High-Power_Motor_Controller_18v15_49FF-7206-7277-5052-4517-1167-if00", baudrate);
    rc[3] = serialport_writebyte(fd[3], (uint8_t)131);

    //fd[4] = serialport_init("/dev/ttyACM4", baudrate);
    fd[4] = serialport_init("/dev/serial/by-id/usb-Pololu_Corporation_Pololu_Simple_High-Power_Motor_Controller_18v15_49FF-7506-7277-5052-2313-1367-if00", baudrate);
    rc[4] = serialport_writebyte(fd[4], (uint8_t)131);

    //fd[5] = serialport_init("/dev/ttyACM5", baudrate);
    fd[5] = serialport_init("/dev/serial/by-id/usb-Pololu_Corporation_Pololu_Simple_High-Power_Motor_Controller_18v15_49FF-7A06-7277-5052-2336-1367-if00", baudrate);
    rc[5] = serialport_writebyte(fd[5], (uint8_t)131);

    return fd;
}

int stop_amp() {
    //stops all the coils
    printf("Stopping amp\n");
    for(int j = 0 ; j < num_amp ; j++)
    {
        rc[j] = serialport_writebyte(fd[j], (uint8_t)224);//stop command
        //required before you can run the motor:
        rc[j] = serialport_writebyte(fd[j], (uint8_t)131);
    }

    return 0;
}

int run_amp(float inpow[]){
    int baudrate = 9600;
    const int buf_max = 256;
    char eolchar = '\n';
    char buf[buf_max];
    int timeout = 1000;
    int speed, speed_byte_1, speed_byte_2;

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
        else if(inpow[j] > -75)
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
            rc[j] = serialport_writebyte(fd[j], 0);
            rc[j] = serialport_writebyte(fd[j], 0);
        }
    }

    usleep(3e3);//sleep for 3 milliseconds

    return 0;
}
