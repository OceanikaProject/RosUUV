#if !defined( I2C_H )
#define I2C_H

#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <sys/types.h>


class I2C
{
    public:

        char *i2c_bus;
        I2C();
        I2C(char*);
        int selectDevice(int, int, char*);
        bool writeR(int, unsigned char *, int);
        void writeRegister(int, int, int);
        int readRegister(int, int);
        bool readBlock(int, unsigned char *, int);
        int i2c_rdwr_block(int, u_int8_t, u_int8_t, u_int8_t, unsigned char*);
};

#endif