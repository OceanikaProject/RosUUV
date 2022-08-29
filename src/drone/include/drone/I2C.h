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
        int  selectDevice(int, int, char*);
        bool i2c_write(int, int);
        void i2c_write_register(int, int, int);
        int  i2c_read_register(int, int);
        bool i2c_read_block(int, unsigned char *, int);
};

#endif