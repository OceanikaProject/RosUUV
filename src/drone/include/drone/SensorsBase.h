#if !defined( SENSORS_BASE_H)
#define SENSORS_BASE_H

#include <math.h>
#include <iostream>
#include <unistd.h>

using namespace std;


class Sensor
{
    public:

        void startup();
        int get_raw_data();

    // protected:

        static int get_value(unsigned int low, unsigned int high)
        {
            /*
            Converting bit value of two 8-bit registers to one
            */

        int value;
        long temp;

        temp = (high << 8) | low;
        if (temp > 32768) temp -= 65536;
        value = static_cast <int> (temp);
        return value;
        }

        static float convert(int value, int range)
        {
            return value / 32768.0 * range;
        }
};


class Sensor3Axis: public Sensor
{
    public:

        long x, y, z;
        int range;

    protected:

        Sensor3Axis()
        {
            this->x = 0;
            this->y = 0;
            this->z = 0;
            this->range = 0;
        }


        void setX(long x, long y, long z)
        {
            this->x = x;
            this->y = y;
            this->z = z;
        }

        // auto getX()
        // {
        //     struct vector
        //     {
        //         long x, y, z;
        //     };
        //     return vector {this->x, this->y, this->z};
        // }

        void set_range(int input_range)
        {
            this->range = input_range;
        }

        int get_range()
        {
            return this->range;
        }
};


class Accelerometer: public Sensor
{
    public:

        long ax, ay, az;
        float axr, ayr, azr;
        int accelerometer_range;



        Accelerometer()
        {
            this->ax = 0;
            this->ay = 0;
            this->az = 0;
            this->accelerometer_range = 0;
        }

    protected:

        void setA(long ax, long ay, long az)
        {
            this->ax = ax;
            this->ay = ay;
            this->az = az;
        }

        // auto getA()
        // {
        //     struct acceleration
        //     {
        //         long ax, ay, az;
        //     };
        //     return acceleration {this->ax, this->ay, this->az};
        // }

        void set_accelerometer_range(int sensor_range)
        {
            this->accelerometer_range = sensor_range;
        }

        int get_accelerometer_range()
        {
            return this->accelerometer_range;
        }

        void get_sample()
        {
            axr = convert(this->ax, this->accelerometer_range);
            ayr = convert(this->ay, this->accelerometer_range);
            azr = convert(this->az, this->accelerometer_range);
        }

        void calibrate(int rounds)
        {
            float axsum = 0.0f, aysum = 0.0f, azsum = 0.0f;
            float axoffset = 0.0f, ayoffset = 0.0f, azoffset = 0.0f;

            std::cout << "Keep accelerometer according to ax = 0 ay = 0 az = 1" << std::endl;

            for (int i; i < rounds; i++)
            {
                get_raw_data();
                get_sample();

                std::cout << "ax = " << axr << " ay = " << ayr << " az = " << azr << endl;

                axsum += axr;
                aysum += ayr;
                azsum += azr;
                usleep(25000);
            }
            axoffset = axsum / rounds;
            ayoffset = aysum / rounds;
            azoffset = azsum / rounds - 1;
        }
};


class Gyroscope: public Sensor
{
    public:
        long gx, gy, gz;
        float gxr, gyr, gzr;
        int gyroscope_range;



        Gyroscope()
        {
            this->gx = 0;
            this->gy = 0;
            this->gz = 0;
            this->gyroscope_range = 0;
        }
    protected:
        void setG(long gx, long gy, long gz)
        {
            this->gx = gx;
            this->gy = gy;
            this->gz = gz;
        }

        // auto getG()
        // {
        //     struct rotation
        //     {
        //         long gx, gy, gz;
        //     };
        //     return rotation {this->gx, this->gy, this->gz};
        // }

        void set_gyroscope_range(int sensor_range)
        {
            this->gyroscope_range = sensor_range;
        }

        int get_gyroscope_range()
        {
            return this->gyroscope_range;
        }

        void get_sample()
        {
            gxr = convert(this->gx, this->gyroscope_range);
            gyr = convert(this->gy, this->gyroscope_range);
            gzr = convert(this->gz, this->gyroscope_range);
        }

        void calibrate(int rounds)
        {
            float gxsum = 0.0f, gysum = 0.0f, gzsum = 0.0f;
            float gxoffset = 0.0f, gyoffset = 0.0f, gzoffset = 0.0f;

            std::cout << "Keep gyroscope steady" << std::endl;

            for (int i; i < rounds; i++)
            {
                get_raw_data();
                get_sample();

                std::cout << "gx = " << gxr << " gy = " << gyr << " gz = " << gzr << endl;

                gxsum += gxr;
                gysum += gyr;
                gzsum += gzr;
                usleep(25000);
            }
            gxoffset = gxsum / rounds;
            gyoffset = gysum / rounds;
            gzoffset = gzsum / rounds;
        }
};


class Magnetometer: public Sensor
{
    public:

        long mx, my, mz;
        float mxr, myr, mzr;
        int magnetometer_range;

    protected:

        Magnetometer()
        {
            this->mx = 0;
            this->my = 0;
            this->mz = 0;
            this->magnetometer_range = 0;
        }

        void setM(long mx, long my, long mz)
        {
            this->mx = mx;
            this->my = my;
            this->mz = mz;
        }

        // auto getM()
        // {
        //     struct magnetic_inclination
        //     {
        //         long mx, my, mz;
        //     };
        //     return magnetic_inclination {this->mx, this->my, this->mz};
        // }

        void set_magnetometer_range(int sensor_range)
        {
            this->magnetometer_range = sensor_range;
        }

        int get_magnetometer_range()
        {
            return this->magnetometer_range;
        }

        void get_sample()
        {
            mxr = convert(this->mx, this->magnetometer_range);
            myr = convert(this->my, this->magnetometer_range);
            mzr = convert(this->mz, this->magnetometer_range);
        }

        void calibrate(int rounds)
        {
            float mxmax = -32768.0f, mymax = -32768.0f, mzmax = -32768.0f;
            float mxmin = 32767.0f, mymin = 32767.0f, mzmin = 32767.0f;
            float mxoffset = 0.0f, myoffset = 0.0f, mzoffset = 0.0f;
            float mxscale = 0.0f, myscale = 0.0f, mzscale = 0.0f;
            float chordx = 0.0f, chordy = 0.0f, chordz = 0.0f;
            float chord_average = 0.0f;

            std::cout << "Rotate compass" << std::endl;

            for (int i = 0; i < rounds; i++)
            {
                get_raw_data();
                get_sample();

                std::cout << "mx = " << mxr << " my = " << myr << " mz = " << mzr << std::endl;

                mxmin = min(mxmin, mxr);
                mymin = min(mymin, myr);
                mzmin = min(mzmin, mzr);

                mxmax = max(mxmax, mxr);
                mymax = max(mymax, myr);
                mzmax = max(mzmax, mzr);
            }
            mxoffset = (mxmax + mxmin) / 2.0f;
            myoffset = (mymax + mymin) / 2.0f;
            mzoffset = (mzmax + mzmin) / 2.0f;

            chordx = (mxmax - mxmin) / 2.0f;
            chordy = (mymax - mymin) / 2.0f;
            chordz = (mzmax - mzmin) / 2.0f;

            chord_average = (chordx + chordy + chordz) / 3;

            mxscale = chord_average / chordx;
            myscale = chord_average / chordy;
            mzscale = chord_average / chordz;
        }
};


class Barometer : public Sensor
{
    public:
        double Pressure;

        Barometer()
        {
            this->Pressure = 0;
        }

        void setP(double pressure)
        {
            this->Pressure = Pressure;
        }

        double getP()
        {
            return this->Pressure;
        }

        float getDepth()
        {
            return static_cast<float>(getP() - 101325) / (9.80665 * 997);
        }

        float getAltitude()
        {
            return static_cast<float>(1 - pow((getP() / 101325), .190295) * 145366.45 * .3048);
        }
};

#endif
