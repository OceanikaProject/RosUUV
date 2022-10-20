#include "drone/sensors.h"


MPU6050::MPU6050() : Accelerometer::Accelerometer(), Gyroscope::Gyroscope()
{
    this->accelerometer_range_configuration = ACCEL_RANGE_2G;
    this->gyroscope_range_configuration = GYRO_RANGE_250DEG;
    this->Accelerometer::set_range(CONVERT_2G);
    this->Gyroscope::set_range(CONVERT_250DEG);
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

    this->Accelerometer::set_range(chosen_accelerometer_range);
    this->Gyroscope::set_range(chosen_gyroscope_range);
}

void MPU6050::startup(I2C bus, int fd)
{
    this->bus = bus;
    this->fd = fd;
    this->bus.selectDevice(this->fd, MPU6050_ADDR, "MPU6050");

    this->bus.i2c_write_register(this->fd, DIV, 7);
    this->bus.i2c_write_register(this->fd, PWR_M, 0);
    this->bus.i2c_write_register(this->fd, CONFIG, 0);
    this->bus.i2c_write_register(this->fd, GYRO_CONFIG, this->gyroscope_range_configuration);
    this->bus.i2c_write_register(this->fd, ACCEL_CONFIG, this->accelerometer_range_configuration);
    this->bus.i2c_write_register(this->fd, INT_EN, 1);
}

void MPU6050::get_binary_data()
{
    this->bus.selectDevice(this->fd, MPU6050_ADDR, "QMC5883");

    float x, y, z;

    x = get_16bit_value(bus.i2c_read_register(fd, ACCEL_XOUT_L), bus.i2c_read_register(fd, ACCEL_XOUT_H));
    y = get_16bit_value(bus.i2c_read_register(fd, ACCEL_YOUT_L), bus.i2c_read_register(fd, ACCEL_YOUT_H));
    z = get_16bit_value(bus.i2c_read_register(fd, ACCEL_ZOUT_L), bus.i2c_read_register(fd, ACCEL_ZOUT_H));

    Accelerometer::set_3d_mgnitude(x, y, z);

    x = get_16bit_value(bus.i2c_read_register(fd, GYRO_XOUT_L), bus.i2c_read_register(fd, GYRO_XOUT_H));
    y = get_16bit_value(bus.i2c_read_register(fd, GYRO_YOUT_L), bus.i2c_read_register(fd, GYRO_YOUT_H));
    z = get_16bit_value(bus.i2c_read_register(fd, GYRO_ZOUT_L), bus.i2c_read_register(fd, GYRO_ZOUT_H));

    Gyroscope::set_3d_mgnitude(x, y, z);
}

void MPU6050::calibration(int rounds)
{
    float x, y, z;

    usleep(4000000);

    float axsum = 0.0f, aysum = 0.0f, azsum = 0.0f;
    float axoffset = 0.0f, ayoffset = 0.0f, azoffset = 0.0f;

    std::cout << "Keep accelerometer according to ax = 0 ay = 0 az = 1" << std::endl;

    for (int i; i < rounds; i++)
    {
        MPU6050::get_binary_data();
        Accelerometer::get_sample();
        Accelerometer::get_3d_magnitude(x, y, z);

        std::cout << "ax = " << x << " ay = " << y << " az = " << z << std::endl;

        axsum += x;
        aysum += y;
        azsum += z;
        usleep(50000);
    }
    axoffset = axsum / rounds;
    ayoffset = aysum / rounds;
    azoffset = azsum / rounds - 1;

    std::cout << "axoffset = " << axoffset << " ayoffset = " << ayoffset << " azoffset = " << azoffset << std::endl;

    usleep(4000000);

    float gxsum = 0.0f, gysum = 0.0f, gzsum = 0.0f;
    float gxoffset = 0.0f, gyoffset = 0.0f, gzoffset = 0.0f;

    std::cout << "Keep gyroscope steady" << std::endl;

    for (int i; i < rounds; i++)
    {
        MPU6050::get_binary_data();
        Gyroscope::get_sample();
        Gyroscope::get_3d_magnitude(x, y, z);

        std::cout << "gx = " << x << " gy = " << y << " gz = " << z << std::endl;

        gxsum += x;
        gysum += y;
        gzsum += z;
        usleep(50000);
    }
    gxoffset = gxsum / rounds;
    gyoffset = gysum / rounds;
    gzoffset = gzsum / rounds;

    std::cout << "axoffset = " << axoffset << " ayoffset = " << ayoffset << " azoffset = " << azoffset << std::endl;
        
}


QMC5883::QMC5883() : Magnetometer::Magnetometer()
{
    this->oversampling = CONFIG_OS512;
    this->range = CONFIG_2GAUSS;
    this->rate = CONFIG_100HZ;
    this->mode = CONFIG_CONT;
    this->set_range(CONVERT_2GAUSS);
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

    this->Magnetometer::set_range(chosen_range);
}


void QMC5883::startup(I2C bus, int fd)
{
    this->bus = bus;
    this->fd = fd;

    this->bus.selectDevice(this->fd, QMC5883_ADDRESS, "QMC5883");

    // this->bus.writeRegister(this->fd, REG_CONTROL_1, oversampling | range | rate | mode);
    this->bus.i2c_write_register(this->fd, REG_CONTROL_1, 0x01);
    // this->bus.writeRegister(this->fd, REG_CONTROL_2, CONFIG2_ROL_PTR);
    this->bus.i2c_write_register(this->fd, REG_CONTROL_2, 0x50);
    this->bus.i2c_write_register(this->fd, REG_PERIOD, 0x01);
}


char QMC5883::read_status()
{
    return bus.i2c_read_register(fd, REG_STATUS);
}


void QMC5883::get_binary_data()
{
    this->bus.selectDevice(this->fd, QMC5883_ADDRESS, "QMC5883");

    float x, y, z;

    x = get_16bit_value(bus.i2c_read_register(fd, XOUT_LSB), bus.i2c_read_register(fd, XOUT_MSB));
    y = get_16bit_value(bus.i2c_read_register(fd, YOUT_LSB), bus.i2c_read_register(fd, YOUT_MSB));
    z = get_16bit_value(bus.i2c_read_register(fd, ZOUT_LSB), bus.i2c_read_register(fd, ZOUT_MSB));

    Magnetometer::set_3d_mgnitude(x, y, z);
}


bool MS5837_30BA::startup(I2C bus, int fd)
{

    this->fd = fd;
    bus.selectDevice(fd, MS5837_30BA_ADDRESS, "Pressure");

    int c;
    int rv;
    unsigned char data[2] = {0};
    unsigned char b[2];
    int result;
    bus.i2c_write(fd, RESET);

    usleep(20000);
    
    for (u_int8_t i = 0; i < 7; i++)
    {
        bus.i2c_write(fd, PROM_READ + 2*i);
        bus.i2c_read_block(fd, data, 2);
        c = (data[1] << 8) | data[0];
        
        c = ((c & 0xFF) << 8) | (c >> 8);
        C[i] = c;
        cout << endl;
    }
    int crc = C[0] >> 12;
    int crcCalculated = _crc4(C);
    if (crcCalculated != crc)
    {
        return false;
    }
    return true;
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


void MS5837_30BA::get_binary_data()
{

    this->bus.selectDevice(this->fd, MS5837_30BA_ADDRESS, "Pressure");

    unsigned long D1 = 0, D2 = 0;
    int rv;
    unsigned char pdata[3] = {0}, tdata[3] = {0};


    bus.i2c_write(fd, CONVERT_D1_OSR8192);
    usleep(2.5e-6*pow(2, (8+5)) * 1000000);

    bus.i2c_write(fd, ADC_READ);
    bus.i2c_read_block(fd, pdata, 3);

    D1 = pdata[0] << 16 | pdata[1] << 8 | pdata[2];

    bus.i2c_write(fd, CONVERT_D2_OSR8192);

    usleep(2.5e-6*pow(2, (8+5)) * 1000000);
    bus.i2c_write(fd, ADC_READ);
    bus.i2c_read_block(fd, tdata, 3);

    D2 = tdata[0] << 16 | tdata[1] << 8 | tdata[2];

    calculate(D1, D2);
}


void MS5837_30BA::calculate(unsigned long D1, unsigned long D2)
{
    int32_t dT, temp;
    int64_t OFF, SENS;
    int32_t SENSi = 0;
    int32_t OFFi = 0;
    int32_t Ti = 0;
    int64_t OFF2 = 0;
    int64_t SENS2 = 0;

    double P, T;

    dT = D2 - u_int32_t(C[5]) * 256l;
    SENS = static_cast<int64_t>(C[1]) * 32768l + (static_cast<int64_t>(C[3]) * dT) / 256l;
    OFF = static_cast<int64_t>(C[2]) * 65536l + (static_cast<int64_t>(C[4]) * dT) / 128l;

    P = (D1 * SENS / 2097152l - OFF) / 8192l;
    T = 2000l + int64_t(dT) * C[6] / 8388608LL;

    if (T / 100. < 20)
    {
        Ti = (3 * static_cast<int64_t>(dT) * static_cast<int64_t>(dT)) / 8589934592LL;
        OFFi = (3 * (T - 2000) * (T - 2000)) / 2;
        SENSi = (5 * (T - 2000) * (T - 2000)) / 8;
        if (T / 100. < -15)
        {
            OFFi = OFFi + 7. * (T + 1500l) * (T + 1500l);
            SENSi = SENSi + 4. * (T + 1500l) * (T + 1500l);
        }
    }
    else
    {
        Ti = 2 * (dT * dT) / (137438953472LL);
        OFFi = (1 * (T - 2000l) * (T - 2000l)) / 16;
        SENSi = 0;
    }
    OFF2 = OFF - OFFi;
    SENS2 = SENS - SENSi;

    T = static_cast<double>( (T - Ti) / 100 );
    P = static_cast<double>( (((D1 * SENS2) / (2097152l) - OFF2) / 8192l) / 10.0 );

    setP(P * conversion);
    setT(T);

}
