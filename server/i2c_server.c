#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
 
// The PiWeather board i2c address
#define ADDRESS 0x04
#define ROUND_TRIP_TIME 10000
 
// The I2C bus: This is for V2 pi's. For V1 Model B you need i2c-0
static const char *devName = "/dev/i2c-1";

int sendType(int argc, char** argv, int file) {
    int val;
    unsigned char cmd[16];

    if (0 == sscanf(argv[1], "%d", &val)) {
        fprintf(stderr, "Invalid parameter %d \"%s\"\n", 1, argv[1]);
        exit(1);
    }

    printf("Sending %d\n", val);
    cmd[0] = val;
     if (write(file, cmd, 1) == 1) {
        usleep(ROUND_TRIP_TIME);
        char buf[1];
        if (read(file, buf, 1) == 1) {
            int temp = (int) buf[0];
            printf("Received %d\n", temp);
        }
    }
    usleep(ROUND_TRIP_TIME);
    return 1;
}

int sendData(int argc, char** argv, int file) {
    int val;
    int i, j;
    unsigned char cmd[16];
    for(i = 2; i < argc; i++) {
        if (0 == sscanf(argv[i], "%d", &val)) {
            fprintf(stderr, "Invalid parameter %d \"%s\"\n", i, argv[i]);
            exit(1);
        }
        printf("Sending %d\n", val);

        for(j = 0; j < 2; j++) {
            cmd[0] = val >> 8 * j;
            if (write(file, cmd, 1) == 1) {
                usleep(ROUND_TRIP_TIME);
                char buf[1];
                if (read(file, buf, 1) == 1) {
                    int temp = (int) buf[0];
                    printf("Received %d\n", temp);
                }
            }
            usleep(ROUND_TRIP_TIME);
        }
    } 
    
    return 1;
}

int init(int argc, char** argv) {
    if (argc == 1) {
        printf("Supply one or more commands to send to the Arduino\n");
        exit(1);
    }

    printf("I2C: Connecting\n");
    int file;
    if ((file = open(devName, O_RDWR)) < 0) {
        //printf(stderr, "I2C: Failed to access %d\n", devName);
        exit(1);
    }
    
    printf("I2C: acquiring buss to 0x%x\n", ADDRESS);
    if (ioctl(file, I2C_SLAVE, ADDRESS) < 0) {
        //printf(stderr, "I2C: Failed to acquire bus access/talk to slave 0x%x\n", ADDRESS);
        exit(1);
    }
    return file;
}

int main(int argc, char** argv) {
    int file = init(argc, argv);
    sendType(argc, argv, file);
    sendData(argc, argv, file);
  
    close(file);
    return EXIT_SUCCESS;
}