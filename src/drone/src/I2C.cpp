#include "drone/I2C.h"


I2C::I2C() {}

I2C::I2C(char *i2c_bus)
{
    this->i2c_bus = i2c_bus;
}


int I2C::selectDevice(int fd, int addr, char *name)
{
    int s;
    char str[128];

    s = ioctl(fd, I2C_SLAVE, addr);

    if (s == -1)
    {
        sprintf(str, "selectDevice for %s", name);
        perror(str);
    }

    return s;
}


bool I2C::i2c_write(int fd, int reg)
{
    unsigned char buf[2] = {reg, 0};
    if (write(fd, buf, 1) != 1) return false;
    return true;
}

bool I2C::i2c_read_block(int fd, unsigned char *buffer, int len)
{
    
    if (read(fd, buffer, len) != len)
    {
        return false;
    }
    return true;
}


void I2C::i2c_write_register(int fd, int reg, int val)
{
    int s;
    char buf[2];

    buf[0]=reg; buf[1]=val;

    s = write(fd, buf, 2);

    if (s == -1)
    {
        perror("writeToDevice");
    }
    else if (s != 2)
    {
        fprintf(stderr, "short write to device\n");
    }
}


int I2C::i2c_read_register(int fd, int reg)
{
    int buf[] = {reg};
    if (write(fd, buf, 1) != 1)
    {
        printf("1");
    }
    if (read(fd, buf, 1) != 1)
    {
        printf("2");
    }
    else
    {
        return buf[0];
    }
}
