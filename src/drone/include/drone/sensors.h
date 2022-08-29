#if !defined( SENSORS_H )
#define SENSORS_H

#include "SensorsBase.h"
#include "I2C.h"


class MPU6050 : public Accelerometer, public Gyroscope
{
    const char PWR_M = 0x6B;
    const char DIV = 0x19;
    const char CONFIG = 0x1A;
    const char GYRO_CONFIG = 0x1B;
    const char ACCEL_CONFIG = 0x1C;
    const char INT_EN = 0x38;
    const char ACCEL_XOUT_H = 0x3B;
    const char ACCEL_XOUT_L = 0x3C;
    const char ACCEL_YOUT_H = 0x3D;
    const char ACCEL_YOUT_L = 0x3E;
    const char ACCEL_ZOUT_H = 0x3F;
    const char ACCEL_ZOUT_L = 0x40;
    const char TEMP_OUT_H = 0x41;
    const char TEMP_OUT_L = 0x42;
    const char GYRO_XOUT_H = 0x43;
    const char GYRO_XOUT_L = 0x44;
    const char GYRO_YOUT_H = 0x45;
    const char GYRO_YOUT_L = 0x46;
    const char GYRO_ZOUT_H = 0x47;
    const char GYRO_ZOUT_L = 0x48;

    const char ACCEL_RANGE_2G = 0x00;
    const char ACCEL_RANGE_4G = 0x08;
    const char ACCEL_RANGE_8G = 0x10;
    const char ACCEL_RANGE_16G = 0x18;

    const char GYRO_RANGE_250DEG = 0x00;
    const char GYRO_RANGE_500DEG = 0x08;
    const char GYRO_RANGE_1000DEG = 0x10;
    const char GYRO_RANGE_2000DEG = 0x18;

    const char MPU6050_ADDR = 0x68;

    enum ACCEL_CONVERSION
    {
        CONVERT_2G = 2,
        CONVERT_4G = 4,
        CONVERT_8G = 8,
        CONVERT_16G = 16
    };

    enum GYRO_CONVERSION
    {
        CONVERT_250DEG = 250,
        CONVERT_500DEG = 500,
        CONVERT_1000DEG = 1000,
        CONVERT_2000DEG = 2000
    };

    I2C bus;
    int fd;
    char accelerometer_range_configuration, gyroscope_range_configuration;

    public:

        MPU6050();

        MPU6050(int chosen_accelerometer_range, int chosen_gyroscope_range);

        void startup(I2C bus, int fd);
        char read_status();
        void get_raw_data();
        void get_sample()
        {
            Accelerometer::get_sample();
            Gyroscope::get_sample();
        }
};


class QMC5883 : public Magnetometer
{
    const char XOUT_LSB = 0x00;
    const char XOUT_MSB = 0x01;
    const char YOUT_LSB = 0x02;
    const char YOUT_MSB = 0x03;
    const char ZOUT_LSB = 0x04;
    const char ZOUT_MSB = 0x05;
    const char REG_STATUS = 0x06;
    const char TOUT_LSB = 0x07;
    const char TOUT_MSB = 0x08;
    const char REG_CONTROL_1 = 0x09;
    const char REG_CONTROL_2 = 0x0A;
    const char REG_PERIOD = 0x0B;
    const char QMC5883_ADDRESS = 0x0D;

    // Oversampling values for the CONFIG register
    const char CONFIG_OS512 = 0b00000000;
    const char CONFIG_OS256 = 0b01000000;
    const char CONFIG_OS128 = 0b10000000;
    const char CONFIG_OS64 = 0b11000000;

    // Range values for the CONFIG register
    const char CONFIG_2GAUSS = 0b00000000;
    const char CONFIG_8GAUSS = 0b00010000;

    // Rate values for the CONFIG register
    const char CONFIG_10HZ = 0b00000000;
    const char CONFIG_50HZ = 0b00000100;
    const char CONFIG_100HZ = 0b00001000;
    const char CONFIG_200HZ = 0b00001100;

    // Mode values for the CONFIG register
    const char CONFIG_STANDBY = 0b00000000;
    const char CONFIG_CONT = 0b00000001;

    // Mode values for the CONFIG2 register
    const char CONFIG2_INT_DISABLE = 0b00000001;
    const char CONFIG2_ROL_PTR = 0b01000000;
    const char CONFIG2_SOFT_RST = 0b10000000;

    public:

        enum MAG_CONVERSION
        {
            CONVERT_2GAUSS = 2,
            CONVERT_8GAUSS = 8
        };

        I2C bus;
        int fd;
        char oversampling, range, rate, mode;

        QMC5883();

        QMC5883(int chosen_oversampling, int chosen_range, int chosen_rate, int chosen_mode);

        void startup(I2C bus, int fd);
        char read_status();
        void get_raw_data();
        void get_sample()
        {
            Magnetometer::get_sample();
        }
};


class MS5837_30BA : public Barometer
{

    const char ADC_READ = 0x00;
    const char RESET = 0x1E;
    const char CONVERT_D1_OSR256 = 0x40;
    const char CONVERT_D1_OSR512 = 0x42;
    const char CONVERT_D1_OSR1024 = 0x44;
    const char CONVERT_D1_OSR2048 = 0x46;
    const char CONVERT_D1_OSR4096 = 0x48;
    const char CONVERT_D1_OSR8192 = 0x4A;
    const char CONVERT_D2_OSR256 = 0x50;
    const char CONVERT_D2_OSR512 = 0x52;
    const char CONVERT_D2_OSR1024 = 0x54;
    const char CONVERT_D2_OSR2048 = 0x56;
    const char CONVERT_D2_OSR4096 = 0x58;
    const char CONVERT_D2_OSR8192 = 0x5A;
    const char PROM_READ = 0xA0;

    const char MS5837_30BA_ADDRESS = 0x76;

    public:

        I2C bus;
        int fd;
        float Temperature;

        const float Pa = 100.0f;
        const float bar = 0.001f;
        const float mbar = 1.0f;
        unsigned int C[8];

        MS5837_30BA() : Barometer::Barometer()
        { }

        bool startup(I2C bus, int fd);
        unsigned char _crc4(unsigned int n_prom[]);


        void get_raw_data();
        void setT(float Temperature)
        {
            this->Temperature = Temperature;
        }

        float getT()
        {
            return Temperature;
        }

        void calculate(unsigned long D1, unsigned long D2);
};


#endif