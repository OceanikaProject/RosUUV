#include "drone/sensors.h"
#include <fcntl.h>
#include <stdlib.h>


#define I2C_BUS "/dev/i2c-1"


int main(int argc, char **argv)
{

   int fd;

   if ((fd = open(I2C_BUS, O_RDWR)) < 0)
   {
       // Open port for reading and writing

       fprintf(stderr, "Failed to open i2c bus %s\n", I2C_BUS);

       exit(1);
   }

   I2C i2c1((char*) I2C_BUS);
    


   MPU6050 imu;
   QMC5883 compass;

   imu.startup(i2c1, fd);
   compass.startup(i2c1, fd);

    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;

    imu.calibration(100);

    // while (1)
    // {
    //     imu.get_raw_data();
    //     compass.get_raw_data();

    //     imu.Accelerometer::get_sample();
    //     imu.Gyroscope::get_sample();
    //     compass.get_sample();

    //     imu.Accelerometer::getX(ax, ay, az);
    //     imu.Gyroscope::getX(gx, gy, gz);
    //     compass.getX(mx, my, mz);

    //     cout << ax << " " << ay << " " << az << " " << gx << " " << gy << " " << gz << endl;

    // }
    return 0;
}
