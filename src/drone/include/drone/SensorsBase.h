#if !defined( SENSORS_BASE_H)
#define SENSORS_BASE_H

#include <math.h>
#include <iostream>
#include <unistd.h>

using namespace std;


class Sensor
{

    /*
        Базовый класс для датчиков
    */

    public:

        virtual void startup() {}
        virtual void get_binary_data() {}
        virtual void calibrate(int) {}

    protected:

        static int get_16bit_value(unsigned int low, unsigned int high)
        {
            /*
                Преобразование старшего и младшего байтов в 16-битное число
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
            /*
                Преобразование данных АЦП в единицу счисления величины
            */
            return value / 32768.0 * range;
        }
};


class Sensor3Axis: public Sensor
{

    /*
        Базовый класс для трехосевых датчиков
    */

    public:

        virtual int get_range() {}
        virtual void get_3d_magnitude(float &x, float &y ,float &z) {}
        virtual void get_sample() {}

    protected:

        virtual void set_3d_mgnitude(float x, float y, float z) {}
        virtual void set_range(int range) {}       
};


class Accelerometer: public Sensor3Axis
{

    /*
        Базовый класс для трехосевых акселерометров
    */

    private:

        float ax, ay, az;
        int range;

    public:

        Accelerometer()
        {
            this->ax = 0;
            this->ay = 0;
            this->az = 0;
            this->range = 0;
        }

        void get_3d_magnitude(float &x, float &y ,float &z)
        {
            x = this->ax;
            y = this->ay;
            z = this->az;
        }

        int get_range()
        {
            return this->range;
        }

        void get_sample()
        {
            ax = convert(this->ax, this->range);
            ay = convert(this->ay, this->range);
            az = convert(this->az, this->range);
        }

    protected:

        void set_3d_mgnitude(float x, float y, float z)
        {
            this->ax = x;
            this->ay = y;
            this->az = z;
        }

        void set_range(int range)
        {
            this->range = range;
        }

        void calibrate(int rounds)
        {

            /*
                Калибровка акселерометра.
                Для калибровки необходимо разместить датчик на поверхности
                параллельной полу и держать его неподвижным.
            */

            float axsum = 0.0f, aysum = 0.0f, azsum = 0.0f;
            float axoffset = 0.0f, ayoffset = 0.0f, azoffset = 0.0f;

            std::cout << "Keep accelerometer according to ax = 0 ay = 0 az = 1" << std::endl;

            for (int i; i < rounds; i++)
            {
                get_binary_data();
                get_sample();

                std::cout << "ax = " << ax << " ay = " << ay << " az = " << az << endl;

                axsum += ax;
                aysum += ay;
                azsum += az;
                usleep(25000);
            }
            axoffset = axsum / rounds;
            ayoffset = aysum / rounds;
            azoffset = azsum / rounds - 1;
        }
};


class Gyroscope: public Sensor3Axis
{

    /*
        Базовый класс для трехосевых гироскопов
    */

    private:

        float gx, gy, gz;
        int range;

    public:

        void get_3d_magnitude(float &x, float &y ,float &z)
        {
            x = this->gx;
            y = this->gy;
            z = this->gz;
        }

        int get_range()
        {
            return this->range;
        }

        void get_sample()
        {
            gx = convert(this->gx, this->range);
            gy = convert(this->gy, this->range);
            gz = convert(this->gz, this->range);
        }

        Gyroscope()
        {
            this->gx = 0;
            this->gy = 0;
            this->gz = 0;
            this->range = 0;
        }

    protected:

        void set_3d_mgnitude(float x, float y, float z)
        {
            this->gx = x;
            this->gy = y;
            this->gz = z;
        }

        void set_range(int range)
        {
            this->range = range;
        }
        
        void calibrate(int rounds)
        {

            /*
                Калибровка гироскопа.
                Для калибровки гироскопа необходимо держать датчик неподвижным.
            */

            float gxsum = 0.0f, gysum = 0.0f, gzsum = 0.0f;
            float gxoffset = 0.0f, gyoffset = 0.0f, gzoffset = 0.0f;

            std::cout << "Keep gyroscope steady" << std::endl;

            for (int i; i < rounds; i++)
            {
                get_binary_data();
                get_sample();

                std::cout << "gx = " << gx << " gy = " << gy << " gz = " << gz << std::endl;

                gxsum += gx;
                gysum += gy;
                gzsum += gz;
                usleep(25000);
            }
            gxoffset = gxsum / rounds;
            gyoffset = gysum / rounds;
            gzoffset = gzsum / rounds;
        }
};


class Magnetometer: public Sensor3Axis
{

    /*
        Базовый класс для трехосевых магнетометров
    */

    private:

        float mx, my, mz;
        int range;

    public:

        void get_3d_magnitude(float &x, float &y ,float &z)
        {
            x = this->mx;
            y = this->my;
            z = this->mz;
        }

        int get_range()
        {
            return this->range;
        }

        void get_sample()
        {
            mx = convert(this->mx, this->range);
            my = convert(this->my, this->range);
            mz = convert(this->mz, this->range);
        }

        Magnetometer()
        {
            this->mx = 0;
            this->my = 0;
            this->mz = 0;
            this->range = 0;
        }

    protected:

        void set_3d_mgnitude(float x, float y, float z)
        {
            this->mx = x;
            this->my = y;
            this->mz = z;
        }      

        void set_range(int range)
        {
            this->range = range;
        }       

        void calibrate(int rounds)
        {

            /*
                Калибровка магнетометра.
                Для калибровки магнетометра необходимо вращать датчик 
                вокруг трех осей во всех направлениях.
            */

            float mxmax = -32768.0f, mymax = -32768.0f, mzmax = -32768.0f;
            float mxmin = 32767.0f, mymin = 32767.0f, mzmin = 32767.0f;
            float mxoffset = 0.0f, myoffset = 0.0f, mzoffset = 0.0f;
            float mxscale = 0.0f, myscale = 0.0f, mzscale = 0.0f;
            float chordx = 0.0f, chordy = 0.0f, chordz = 0.0f;
            float chord_average = 0.0f;

            std::cout << "Rotate compass" << std::endl;

            for (int i = 0; i < rounds; i++)
            {
                get_binary_data();
                get_sample();

                std::cout << "mx = " << mx << " my = " << my << " mz = " << mz << std::endl;

                mxmin = min(mxmin, mx);
                mymin = min(mymin, my);
                mzmin = min(mzmin, mz);

                mxmax = max(mxmax, mx);
                mymax = max(mymax, my);
                mzmax = max(mzmax, mz);
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

    /*
        Базовый класс для барометров
    */

    protected:

        double Pressure;
        void setP(double Pressure)
        {
            this->Pressure = Pressure;
        }

    public:

        Barometer()
        {
            this->Pressure = 0;
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
