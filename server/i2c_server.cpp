#include "i2c_server.h"

/*
 *  sudo apt-get install -y i2c-tools
 *  sudo i2cdetect -l
 *  sudo chmod o+rw /dev/i2c*
 */

static const char *devName = "/dev/i2c-1";
int i2c_file;
pthread_t i2c_thr;
int volatile i2c_type;
int volatile i2c_data;

int sendType(int type) {
    int val;
    unsigned char cmd[16];
    val = type;
#ifdef I2C_DEBUG
    printf("Type %d\n", val);
#endif
    cmd[0] = val;
     if (write(i2c_file, cmd, 1) == 1) {
        usleep(ROUND_TRIP_TIME);
        char buf[1];
        if (read(i2c_file, buf, 1) == 1) {
            int temp = (int) buf[0];
#ifdef I2C_DEBUG
            printf("Received %d\n", temp);
#endif
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
#ifdef I2C_DEBUG
        printf("Data %d\n", val);
#endif

        for(j = 0; j < 2; j++) {
            cmd[0] = val >> 8 * j;
            if (write(i2c_file, cmd, 1) == 1) {
                usleep(ROUND_TRIP_TIME);
                char buf[1];
                if (read(i2c_file, buf, 1) == 1) {
                    int temp = (int) buf[0];
#ifdef I2C_DEBUG
                    printf("Received %d\n", temp);
#endif
                }
            }
            usleep(ROUND_TRIP_TIME);
        }
    } 
    
    return 1;
}

int signal_limiter(int data) {
    int thrs = 30;
    int mid = 90;
    if(data < mid - thrs) {
        return mid - thrs;
    } 
    if(data > mid + thrs) {
        return mid + thrs;
    }
    return data;
}

int i2c_send(int type, int data) {
#ifdef SERVO_PROTECT_MK1
    data = signal_limiter(data);
#endif
    //TODO: need some kind of mutex
    i2c_type = type;
    i2c_data = data;
    return EXIT_SUCCESS;
}

void* i2c_manager(void *ptr) {
    (void) ptr;
    void *sksk = NULL;
#ifdef I2C_DEBUG
    printf("thread created\n");
#endif
    while(1) {
        usleep(I2C_LATENCY);
        sendType(i2c_type);
        sendData(i2c_data);
    }
    //return sksk;
}

int i2c_init() {
    // locate i2c file 
    if ((i2c_file = open(devName, O_RDWR)) < 0) {
#ifdef I2C_DEBUG
        printf("I2C: Failed to access \n");
#endif
        exit(1);
    }
#ifdef I2C_DEBUG
    printf("I2C: acquiring buss to 0x%x\n", ADDRESS);
#endif
    if (ioctl(i2c_file, I2C_SLAVE, ADDRESS) < 0) {
#ifdef I2C_DEBUG
        printf("I2C: Failed to acquire bus access/talk to slave\n");
#endif
        exit(1);
    }

    // create managing thread
    //void* dat = malloc(sizeof(void *));
    int thr_ret = pthread_create(&i2c_thr, NULL, i2c_manager, NULL);
    if (thr_ret) {
#ifdef I2C_DEBUG
        printf("thread not created\n");
#endif
        exit(1);
    }

    return EXIT_SUCCESS;
}

int i2c_exit() {
    close(i2c_file);
    pthread_join(i2c_thr, NULL);
#ifdef I2C_DEBUG
    printf("thread joined\n");
#endif
    return EXIT_SUCCESS;
}


/*
int main(int argc, char** argv) {
    i2c_init();
    
    int itype;
    sscanf(argv[1], "%d", &itype);
    int idata;
    sscanf(argv[2], "%d", &idata);

    i2c_send(itype, idata);
    
    i2c_exit();
    while(1);
    return EXIT_SUCCESS;
}
*/

