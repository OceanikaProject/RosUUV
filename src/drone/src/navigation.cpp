#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "drone/sensors.h"
#include "geometry_msgs/Vector3.h"
#include <fcntl.h>
#include "drone/MadgwickFilter.h"


#define I2C_BUS "/dev/i2c-1"


int main(int argc, char **argv)
{

   int fd;

   if ((fd = open(I2C_BUS, O_RDWR)) < 0)
   {
       // Open port for reading and writing

    //    fprintf(stderr, "Failed to open i2c bus %s\n", I2C_BUS);

       exit(1);
   }

   I2C i2c1((char*) I2C_BUS);

    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<geometry_msgs::PoseStamped>("talker_topic", 1000);
    ros::Rate loop_rate(10);

   MPU6050 imu;
   QMC5883 compass;
   MS5837_30BA bar;
   MadgwickFilter ahrs;

//    imu.startup(i2c1, fd);
//    compass.startup(i2c1, fd);
   bool ok = bar.startup(i2c1, fd);
   cout << "ok = " << ok << endl;

    while (ros::ok())
    {
        // imu.get_raw_data();
        // compass.get_raw_data();
        // bar.get_raw_data();

        // imu.get_sample();
        // compass.get_sample();

        // cout << imu.axr << " " << imu.ayr << " " << imu.azr << " " << imu.gxr << " " << imu.gyr << " " << imu.gzr << endl;
        // cout << compass.mxr << " " << compass.myr << " " << compass.mzr << endl;
        // cout << bar.getP() << " | " << bar.getT() << endl;

        // bar.getDepth();

        // ahrs.update(imu.gxr, imu.gyr, imu.gzr, imu.axr, imu.ayr, imu.azr, compass.mxr, compass.myr, compass.mzr);

        // geometry_msgs::Vector3 msg;
        // msg.x = 1;
        // msg.y = 2;
        // msg.z = 3;

        // geometry_msgs::PoseStamped msg;

        // msg.pose.orientation.w = ahrs.q0;
        // msg.pose.orientation.x = ahrs.q1;
        // msg.pose.orientation.y = ahrs.q2;
        // msg.pose.orientation.z = ahrs.q3;

        // msg.pose.position.x = 0;
        // msg.pose.position.y = 0;
        // msg.pose.position.z = bar.getDepth();

        // pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
