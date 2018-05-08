// g++ -o test amplifier_handshaking.cpp

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

const unsigned char CRC7_POLY = 0x91;

unsigned char getCRC(unsigned char message[], unsigned char length)
{
  unsigned char i, j, crc = 0;

  for (i = 0; i < length; i++)
  {
    crc ^= message[i];
    for (j = 0; j < 8; j++)
    {
      if (crc & 1)
        crc ^= CRC7_POLY;
      crc >>= 1;
    }
  }
  return crc;
}

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

int main (int argc, char * argv[]) {
    int baudrate = 9600;
    const int buf_max = 256;
    char eolchar = '\n';
    char buf[buf_max];
    int timeout = 1000;
    int speed, speed_byte_1, speed_byte_2;

    //number of amplifiers
    int num_amp = 6;

    //input power (percentage)
    double inpow[] = {0,0,0,0,0,0};

    int fd[num_amp] = {0};
    int rc[num_amp] = {0};

    /*char **optarg = (char **)malloc(sizeof(int)*num_amp);

    for(int i = 0 ; i < num_amp ; i++)
    {
        optarg[i] = (char *)malloc(sizeof(char)*13);
    }

    sprintf(optarg[1], "/dev/ttyACM1");
    printf("here2\n");
    sprintf(optarg[2], "/dev/ttyACM2");
    printf("here3\n");
    sprintf(optarg[3], "/dev/ttyACM3");
    printf("here4\n");
    sprintf(optarg[4], "/dev/ttyACM4");
    printf("here5\n");
    sprintf(optarg[5], "/dev/ttyACM5");
    printf("here6\n");
    sprintf(optarg[0], "/dev/ttyACM0");
    printf("here1\n");
        struct timeval start;
        double time_initial, time_current, time_last, time_elapsed, time_out;
        gettimeofday(&start, NULL);
        time_initial = (double) start.tv_sec+start.tv_usec*1e-6;*/
    //initialization for the motors

    fd[0] = serialport_init("/dev/ttyUSB0", baudrate);

    //fd[0] = serialport_init("/dev/serial/by-id/usb-Pololu_Corporation_Pololu_Simple_Motor_Controller_18v7_52FF-6E06-7283-5255-2916-2567-if00", baudrate);
    //rc[0] = serialport_writebyte(fd[0], (uint8_t)131);

    //fd[1] = serialport_init("/dev/ttyACM1", baudrate);
    //fd[1] = serialport_init("/dev/serial/by-id/usb-Pololu_Corporation_Pololu_Simple_Motor_Controller_18v7_49FF-7206-7277-5052-2531-1367-if00", baudrate);
    //rc[1] = serialport_writebyte(fd[1], (uint8_t)131);

    //fd[2] = serialport_init("/dev/ttyACM2", baudrate);
    //fd[2] = serialport_init("/dev/serial/by-id/usb-Pololu_Corporation_Pololu_Simple_High-Power_Motor_Controller_18v15_49FF-6D06-7277-5052-3251-0967-if00", baudrate);
    //rc[2] = serialport_writebyte(fd[2], (uint8_t)131);

    //fd[3] = serialport_init("/dev/ttyACM3", baudrate);
    // fd[3] = serialport_init("/dev/serial/by-id/usb-Pololu_Corporation_Pololu_Simple_High-Power_Motor_Controller_18v15_49FF-7206-7277-5052-4517-1167-if00", baudrate);
    // rc[3] = serialport_writebyte(fd[3], (uint8_t)131);

    //fd[4] = serialport_init("/dev/ttyACM4", baudrate);
    // fd[4] = serialport_init("/dev/serial/by-id/usb-Pololu_Corporation_Pololu_Simple_High-Power_Motor_Controller_18v15_49FF-7506-7277-5052-2313-1367-if00", baudrate);
    // rc[4] = serialport_writebyte(fd[4], (uint8_t)131);

    //fd[5] = serialport_init("/dev/ttyACM5", baudrate);
    // fd[5] = serialport_init("/dev/serial/by-id/usb-Pololu_Corporation_Pololu_Simple_High-Power_Motor_Controller_18v15_49FF-7A06-7277-5052-2336-1367-if00", baudrate);
    // rc[5] = serialport_writebyte(fd[5], (uint8_t)131);

    struct timeval start;
    double time_initial, time_current, time_last, time_elapsed, time_out;
    gettimeofday(&start, NULL);
    time_initial = (double) start.tv_sec+start.tv_usec*1e-6;

    time_current = 0;

    double freq = 0.1;

    double input;

    //rc[0] = serialport_writebyte(fd[0], (uint8_t)131);

    //serialport_flus(fd[0]);
    //printf("after flush 1\n");



    for(int j = 0 ; j < num_amp ; j++) {
        unsigned char message[4] = {170, j+1, 3, 0x00};
        message[3] = getCRC(message, 3);

        //printf("%d\n", j);
        //fd[j] = serialport_init(optarg[j], baudrate);
        //printf("amplifier: %d, fd: %d\n", j, fd[j]);
        rc[j] = serialport_writebyte(fd[0], (uint8_t)message[0]);  // pololu protocal
        //printf("rc %d\n", rc[j]);
        rc[j] = serialport_writebyte(fd[0], (uint8_t)message[1]);    // device number
        //printf("rc %d\n", rc[j]);

        rc[j] = serialport_writebyte(fd[0], (uint8_t)message[2]);
        rc[j] = serialport_writebyte(fd[0], (uint8_t)message[3]);
        //printf("rc %d\n", rc[j]);
        usleep(3e3);
    }

    while(time_current < 5) {
        //input = 5*sin(2*3.14159*freq*time_current)*1.1;
        input = 12;

        inpow[2] = input;
        inpow[3] = input;
        // inpow[5] = input;

        //serialport_flus(fd[0]);
        //printf("after flush 2\n");
        //testing with speed (voltage)
        for(int j = 0 ; j < num_amp ; j++)
        {

            time_elapsed = time_last-time_initial;
            if(inpow[j] > 0)
            {

            //gettimeofday(&start, NULL);
            //double time_initial = (double) start.tv_sec+start.tv_usec*1e-6;

                unsigned char message[4] = {170, j+1, 3, 0x00};
                message[3] = getCRC(message, 3);

                speed = (inpow[j]/100)*3200;
                speed_byte_1 = speed % 32;
                speed_byte_2 = speed / 32;
                rc[j] = serialport_writebyte(fd[0], (uint8_t)message[0]);  // pololu protocal
                rc[j] = serialport_writebyte(fd[0], (uint8_t)message[1]);    // device number
                rc[j] = serialport_writebyte(fd[0], (uint8_t)message[2]);
                rc[j] = serialport_writebyte(fd[0], (uint8_t)message[3]);
                usleep(3e3);

                unsigned char message2[6] = {170, j+1, 5, speed_byte_1, speed_byte_2, 0x00};
                message2[5] = getCRC(message2, 5);

                rc[j] = serialport_writebyte(fd[0], (uint8_t)message2[0]);  // pololu protocal
                rc[j] = serialport_writebyte(fd[0], (uint8_t)message2[1]);    // device number
                rc[j] = serialport_writebyte(fd[0], (uint8_t)message2[2]);
                rc[j] = serialport_writebyte(fd[0], (uint8_t)message2[3]);
                rc[j] = serialport_writebyte(fd[0], (uint8_t)message2[4]);
                rc[j] = serialport_writebyte(fd[0], (uint8_t)message2[5]);
                usleep(3e3);

            //gettimeofday(&start, NULL);
            //time_last = (double) start.tv_sec + start.tv_usec*1e-6;

            //printf("%f", time_last-time_initial);
                }
                else
                {
                    unsigned char message[4] = {170, j+1, 3, 0x00};
                    message[3] = getCRC(message, 3);

                speed = (inpow[j]/-100)*3200;
                speed_byte_1 = speed % 32;
                speed_byte_2 = speed / 32;
                rc[j] = serialport_writebyte(fd[0], (uint8_t)message[0]);  // pololu protocal
                rc[j] = serialport_writebyte(fd[0], (uint8_t)message[1]);    // device number
                rc[j] = serialport_writebyte(fd[0], (uint8_t)message[2]);
                rc[j] = serialport_writebyte(fd[0], (uint8_t)message[3]);
                usleep(3e3);
                unsigned char message2[6] = {170, j+1, 6, speed_byte_1, speed_byte_2, 0x00};
                message2[5] = getCRC(message2, 5);

                rc[j] = serialport_writebyte(fd[0], (uint8_t)message2[0]);  // pololu protocal
                rc[j] = serialport_writebyte(fd[0], (uint8_t)message2[1]);    // device number
                rc[j] = serialport_writebyte(fd[0], (uint8_t)message2[2]);
                rc[j] = serialport_writebyte(fd[0], (uint8_t)message2[3]);
                rc[j] = serialport_writebyte(fd[0], (uint8_t)message2[4]);
                rc[j] = serialport_writebyte(fd[0], (uint8_t)message2[5]);
                usleep(3e3);
                }
        }

        // usleep(3e3);//sleep for 1 seconds

        gettimeofday(&start, NULL);
        time_current = (double) start.tv_sec+start.tv_usec*1e-6 - time_initial;

    }

    // usleep(1e6);//sleep for 10 seconds

    //printf("after setting signal\n");
    //stops all the coils
    // for(int j = 0 ; j < num_amp ; j++)
    // {
    //     unsigned char message[4] = {170, j+1, 96, 0x00};
    //     message[3] = getCRC(message, 3);
    //
    //     rc[j] = serialport_writebyte(fd[0], (uint8_t)message[0]);  // pololu protocal
    //     rc[j] = serialport_writebyte(fd[0], (uint8_t)message[1]);    // device number
    //     rc[j] = serialport_writebyte(fd[0], (uint8_t)message[2]);//stop command
    //     rc[j] = serialport_writebyte(fd[0], (uint8_t)message[3]);//stop command
    //     usleep(3e3);
    //     //printf("stop amplifier %d\n", j);
    //     //required before you can run the motor:
    //
    //     unsigned char message2[4] = {170, j+1, 3, 0x00};
    //     message2[3] = getCRC(message2, 3);
    //
    //     rc[j] = serialport_writebyte(fd[0], (uint8_t)message2[0]);  // pololu protocal
    //     rc[j] = serialport_writebyte(fd[0], (uint8_t)message2[1]);    // device number
    //     rc[j] = serialport_writebyte(fd[0], (uint8_t)message2[2]);//stop command
    //     rc[j] = serialport_writebyte(fd[0], (uint8_t)message2[3]);//stop command
    //     usleep(3e3);
    // }
    //
    // // testing to recieve the temperature
    // // for(int j = 0 ; j < num_amp ; j++)
    // {
    //     //printf("%d\n",(sizeof(optarg)/sizeof(*optarg)));
    //     printf("initialization result is %d.\n", fd[j]);
    //     //serialport_flus(fd);
    //     printf("before writing.\n");
    //     rc[j] = serialport_writebyte(fd[j], (uint8_t)161);
    //     printf("after writing.\n");
    //     //serialport_flus(fd);
    //     printf("before writing.\n");
    //     rc[j] = serialport_writebyte(fd[j], (uint8_t)24);
    //     printf("after writing.\n");
    //     if (rc[j] == -1)   error("error in writing.\n");
    //
    //     memset(buf, 0, buf_max);
    //     printf("before reading.\n");
    //     rc[j] = serialport_read_until(fd[j], buf, eolchar, buf_max, timeout);
    //     printf("read string:");
    //     printf("%s.\n", buf);
    //     for (int i = 0; i < 2; i ++) {
    //     printf("  %d  ", buf[i]);
    //     }
    //     printf("reading result code %d.\n", rc[j]);
    // }
}
