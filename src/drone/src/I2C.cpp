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


void I2C::writeRegister(int fd, int reg, int val)
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


int I2C::readRegister(int fd, int reg)
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


int I2C::i2c_rdwr_block(int fd, u_int8_t reg, u_int8_t read_write, u_int8_t length, unsigned char* buffer)
{
    struct i2c_smbus_ioctl_data ioctl_data;
    union i2c_smbus_data smbus_data;

    int rv;

    if (length > I2C_SMBUS_BLOCK_MAX)
    {
        return -1;
    }

    smbus_data.block[0] = length;

    if (read_write != I2C_SMBUS_READ)
    {
        for (int i = 0; i < length; i++)
        {
            smbus_data.block[i + 1] = buffer[i];
        }
    }

    ioctl_data.read_write = read_write;
    ioctl_data.command = reg;
    ioctl_data.size = I2C_SMBUS_I2C_BLOCK_DATA;
    ioctl_data.data = &smbus_data;

    rv = ioctl(fd, I2C_SMBUS, &ioctl_data);

    if (rv < 0)
    {
        return rv;
    }

    if (read_write == I2C_SMBUS_READ)
    {
        for (int i = 0; i < length; i++)
        {
            buffer[i] = smbus_data.block[i + 1];
        }
    }

    return rv;
}