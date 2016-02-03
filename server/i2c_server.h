#ifndef __I2C_SERVER_H__
#define __I2C_SERVER_H__

#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

#define ADDRESS 0x04
#define ROUND_TRIP_TIME 10000

int i2c_send(int type, int data);
int i2c_init(void);

#endif