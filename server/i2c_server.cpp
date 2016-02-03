#include "i2c_server.h"
// The I2C bus: This is for V2 pi's. For V1 Model B you need i2c-0
static const char *devName = "/dev/i2c-1";
int i2c_file;

int sendType(int type) {
    int val;
    unsigned char cmd[16];
    val = type;

    //printf("Sending %d\n", val);
    cmd[0] = val;
     if (write(i2c_file, cmd, 1) == 1) {
        usleep(ROUND_TRIP_TIME);
        char buf[1];
        if (read(i2c_file, buf, 1) == 1) {
            int temp = (int) buf[0];
            //printf("Received %d\n", temp);
        }
    }
    usleep(ROUND_TRIP_TIME);
    return 1;
}

int sendData(int data) {
    int val;
    int i, j;
    unsigned char cmd[16];
    val = data;
    for(i = 0; i < 1; i++) {
        //printf("Sending %d\n", val);

        for(j = 0; j < 2; j++) {
            cmd[0] = val >> 8 * j;
            if (write(i2c_file, cmd, 1) == 1) {
                usleep(ROUND_TRIP_TIME);
                char buf[1];
                if (read(i2c_file, buf, 1) == 1) {
                    int temp = (int) buf[0];
                    //printf("Received %d\n", temp);
                }
            }
            usleep(ROUND_TRIP_TIME);
        }
    } 
    
    return 1;
}

int i2c_send(int type, int data) {
    sendType(type);
    sendData(data);
    return 1;
}

int i2c_init() {
    if ((i2c_file = open(devName, O_RDWR)) < 0) {
        //printf(stderr, "I2C: Failed to access %d\n", devName);
        exit(1);
    }
    
    printf("I2C: acquiring buss to 0x%x\n", ADDRESS);
    if (ioctl(i2c_file, I2C_SLAVE, ADDRESS) < 0) {
        //printf(stderr, "I2C: Failed to acquire bus access/talk to slave 0x%x\n", ADDRESS);
        exit(1);
    }
    return i2c_file;
}

/*
int main(int argc, char** argv) {
    i2c_init();
    i2c_send(2, 10);
    close(i2c_file);
    return EXIT_SUCCESS;
}
*/