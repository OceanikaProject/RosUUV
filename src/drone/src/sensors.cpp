#include "drone/sensors.h"


MPU6050::MPU6050() : Accelerometer::Accelerometer(), Gyroscope::Gyroscope()
{
    this->accelerometer_range_configuration = ACCEL_RANGE_2G;
    this->gyroscope_range_configuration = GYRO_RANGE_250DEG;
    this->set_accelerometer_range(CONVERT_2G);
    this->set_gyroscope_range(CONVERT_250DEG);
}

MPU6050::MPU6050(int chosen_accelerometer_range, int chosen_gyroscope_range) : Accelerometer::Accelerometer(), Gyroscope::Gyroscope()
{

    if (chosen_accelerometer_range == CONVERT_2G)
    {
        this->accelerometer_range_configuration = ACCEL_RANGE_2G;
    }
    else if (chosen_accelerometer_range == CONVERT_4G)
    {
        this->accelerometer_range_configuration = ACCEL_RANGE_4G;
    }
    else if (chosen_accelerometer_range == CONVERT_8G)
    {
        this->accelerometer_range_configuration = ACCEL_RANGE_8G;
    }
    else if (chosen_accelerometer_range == CONVERT_16G)
    {
        this->accelerometer_range_configuration = ACCEL_RANGE_16G;
    }

    if (chosen_gyroscope_range == CONVERT_250DEG)
    {
        this->gyroscope_range_configuration = GYRO_RANGE_250DEG;
    }
    else if (chosen_gyroscope_range == CONVERT_500DEG)
    {
        this->gyroscope_range_configuration = GYRO_RANGE_500DEG;
    }
    else if (chosen_gyroscope_range == CONVERT_1000DEG)
    {
        this->gyroscope_range_configuration = GYRO_RANGE_1000DEG;
    }
    else if (chosen_gyroscope_range == CONVERT_2000DEG)
    {
        this->gyroscope_range_configuration = GYRO_RANGE_2000DEG;
    }

    this->set_accelerometer_range(chosen_accelerometer_range);
    this->set_gyroscope_range(chosen_gyroscope_range);
}

void MPU6050::startup(I2C bus, int fd)
{
    this->bus = bus;
    this->fd = fd;
    this->bus.selectDevice(this->fd, MPU6050_ADDR, "MPU6050");

    this->bus.writeRegister(this->fd, DIV, 7);
    this->bus.writeRegister(this->fd, PWR_M, 1);
    this->bus.writeRegister(this->fd, CONFIG, 0);
    this->bus.writeRegister(this->fd, GYRO_CONFIG, this->gyroscope_range_configuration);
    this->bus.writeRegister(this->fd, ACCEL_CONFIG, this->accelerometer_range_configuration);
    this->bus.writeRegister(this->fd, INT_EN, 1);
}

int MPU6050::get_raw_data()
{
    this->bus.selectDevice(this->fd, MPU6050_ADDR, "QMC5883");

    ax = get_value(bus.readRegister(fd, ACCEL_XOUT_L), bus.readRegister(fd, ACCEL_XOUT_H));
    ay = get_value(bus.readRegister(fd, ACCEL_YOUT_L), bus.readRegister(fd, ACCEL_YOUT_H));
    az = get_value(bus.readRegister(fd, ACCEL_ZOUT_L), bus.readRegister(fd, ACCEL_ZOUT_H));

    gx = get_value(bus.readRegister(fd, GYRO_XOUT_L), bus.readRegister(fd, GYRO_XOUT_H));
    gy = get_value(bus.readRegister(fd, GYRO_YOUT_L), bus.readRegister(fd, GYRO_YOUT_H));
    gz = get_value(bus.readRegister(fd, GYRO_ZOUT_L), bus.readRegister(fd, GYRO_ZOUT_H));
    return 0;
}


QMC5883::QMC5883() : Magnetometer::Magnetometer()
{
    this->oversampling = CONFIG_OS512;
    this->range = CONFIG_2GAUSS;
    this->rate = CONFIG_100HZ;
    this->mode = CONFIG_CONT;
    this->set_magnetometer_range(CONVERT_2GAUSS);
}


QMC5883::QMC5883( int chosen_oversampling, int chosen_range, int chosen_rate, int chosen_mode) : Magnetometer::Magnetometer()
{

    if (chosen_oversampling == 512)
    {
        this->oversampling = CONFIG_OS512;
    }
    else if (chosen_oversampling == 256)
    {
        this->oversampling = CONFIG_OS256;
    }
    else if (chosen_oversampling == 128)
    {
        this->oversampling = CONFIG_OS128;
    }
    else if (chosen_oversampling == 64)
    {
        this->oversampling = CONFIG_OS64;
    }


    if (chosen_range == CONVERT_2GAUSS)
    {
        this->range = CONFIG_2GAUSS;
    }
    else if (chosen_range == CONVERT_8GAUSS)
    {
        this->range = CONFIG_8GAUSS;
    }


    if (chosen_rate == 10)
    {
        this->rate = CONFIG_10HZ;
    }
    else if (chosen_rate == 50)
    {
        this->rate = CONFIG_50HZ;
    }
    else if (chosen_rate == 100)
    {
        this->rate = CONFIG_100HZ;
    }
    else if (chosen_rate == 200)
    {
        this->rate = CONFIG_200HZ;
    }

    if (chosen_mode == 0) 
    {
        this->mode = CONFIG_STANDBY;
    }
    else if (chosen_mode == 1) 
    {
        this->mode = CONFIG_CONT;
    }

    this->set_magnetometer_range(chosen_range);
}


void QMC5883::startup(I2C bus, int fd)
{
    this->bus = bus;
    this->fd = fd;

    this->bus.selectDevice(this->fd, QMC5883_ADDRESS, "QMC5883");

    // this->bus.writeRegister(this->fd, REG_CONTROL_1, oversampling | range | rate | mode);
    this->bus.writeRegister(this->fd, REG_CONTROL_1, 0x01);
    // this->bus.writeRegister(this->fd, REG_CONTROL_2, CONFIG2_ROL_PTR);
    this->bus.writeRegister(this->fd, REG_CONTROL_2, 0x50);
    this->bus.writeRegister(this->fd, REG_PERIOD, 0x01);
}


char QMC5883::read_status()
{
    return bus.readRegister(fd, REG_STATUS);
}


int QMC5883::get_raw_data()
{
    this->bus.selectDevice(this->fd, QMC5883_ADDRESS, "QMC5883");

    mx = get_value(bus.readRegister(fd, XOUT_LSB), bus.readRegister(fd, XOUT_MSB));
    my = get_value(bus.readRegister(fd, YOUT_LSB), bus.readRegister(fd, YOUT_MSB));
    mz = get_value(bus.readRegister(fd, ZOUT_LSB), bus.readRegister(fd, ZOUT_MSB));
}


void MS5837_30BA::startup(I2C bus, int fd)
{
    bus.selectDevice(fd, MS5837_30BA_ADDRESS, "Pressure");

    int c;

    bus.writeRegister(fd, RESET, 0);

    usleep(10000);

    for (u_int8_t i = 0; i < 7; i++)
    {
        c = bus.readRegister(fd, PROM_READ + 2*i);
        c = ((c & 0xFF) << 8) | (c >> 8);
        C[i] = c;

    }
    int crc = C[0] >> 12;
    int crcCalculated = _crc4(C);
    // if (crcCalculated != crc)
    // {
    //     return false;
    // }
    // return true;
}


unsigned char MS5837_30BA::_crc4(unsigned int n_prom[])
{
    int cnt;
    unsigned int n_rem = 0;
    unsigned char n_bit;

    n_prom[0] = ((n_prom[0]) & 0x0FFF);
    n_prom[7] = 0;
    for (cnt = 0; cnt < 16; cnt++)
    {
        if (cnt%2==1) n_rem ^= (unsigned short) ((n_prom[cnt>>1]) & 0x00FF);
        else n_rem ^= (unsigned short) (n_prom[cnt>>1]>>8);
        for (n_bit = 8; n_bit > 0; n_bit--)
        {
            if (n_rem & (0x8000)) n_rem = (n_rem << 1) ^ 0x3000;
            else n_rem = (n_rem << 1);
        }
    }
    n_rem = ((n_rem >> 12) & 0x000F);
    return (n_rem ^ 0x00);
}


int MS5837_30BA::get_raw_data()
{

    // this->bus.selectDevice(this->fd, MS5837_30BA_ADDRESS, "Pressure");

    unsigned long D1 = 0, D2 = 0;
    int rv;
    unsigned char pdata[3] = {0}, tdata[3] = {0};
    bus.writeRegister(fd, CONVERT_D1_OSR8192, 0);
    usleep(2.5e-6*pow(2, (8+5)));
    rv = bus.i2c_rdwr_block(fd, ADC_READ, I2C_SMBUS_READ, 3, pdata);
    D1 = pdata[0] << 16 | pdata[1] << 8 | pdata[2];

    bus.writeRegister(fd, CONVERT_D2_OSR8192, 0);
    usleep(2.5e-6*pow(2, (8+5)));
    rv = bus.i2c_rdwr_block(fd, ADC_READ, I2C_SMBUS_READ, 3, tdata);
    D2 = tdata[0] << 16 | tdata[1] << 8 | tdata[2];

    calculate(D1, D2);
    return 0;
}


void MS5837_30BA::calculate(int D1, int D2)
{
    long dT, temp;
    long long OFF, SENS;
    long SENSi = 0;
    long long OFFi = 0;
    long Ti = 0;
    long OFF2 = 0;
    long SENS2 = 0;

    dT = D2 - u_int32_t(C[5]) * 256l;
    SENS = static_cast<long long>(C[1]) * 32768l + (static_cast<long long>(C[3]) * dT) / 256l;
    OFF = static_cast<long long>(C[2]) * 65536l + (int64_t(C[4]) * dT) / 128l;

    Pressure = (D1 * SENS / 2097152l - OFF) / 8192l;
    Temperature = 2000l + int64_t(dT) * C[6] / 8388608LL;

    if (Temperature / 100. < 20)
    {
        Ti = (3 * static_cast<long long>(dT) * static_cast<long long>(dT)) / 8589934592LL;
        OFFi = (3 * (Temperature - 2000) * (Temperature - 2000)) / 2;
        SENSi = (5 * (Temperature - 2000) * (Temperature - 2000)) / 8;
        if (Temperature / 100. < -15)
        {
            OFFi = OFFi + 7. * (Temperature + 1500l) * (Temperature + 1500l);
            SENSi = SENSi + 4. * (Temperature + 1500l) * (Temperature + 1500l);
        }
    }
    else
    {
        Ti = 2 * (dT * dT) / (137438953472LL);
        OFFi = (1 * (Temperature - 2000l) * (Temperature - 2000l)) / 16;
        SENSi = 0;
    }
    OFF2 = OFF - OFFi;
    SENS2 = SENS - SENSi;

    Temperature = (Temperature - Ti) / 100;
    Pressure = (((D1 * SENS2) / (2097152l) - OFF2) / 8192l) / 10.0;
}
