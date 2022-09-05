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

       fprintf(stderr, "Failed to open i2c bus %s\n", I2C_BUS);

       exit(1);
   }

   I2C i2c1((char*) I2C_BUS);

    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<geometry_msgs::PoseStamped>("talker_topic", 1000);
    ros::Rate loop_rate(100);

    const bool DEBUG = n.hasParam("/talker_node/DEBUG");
    const bool CALIBRATED = n.hasParam("/talker_node/CALIBRATED");
    

    static ros::Publisher acceleration_pub;
    static ros::Publisher rotation_pub;
    static ros::Publisher magnetism_pub;
    static ros::Publisher euler_pub;
    

   if (DEBUG)
   {
    acceleration_pub = n.advertise<geometry_msgs::Vector3>("acceleration", 1000);
    rotation_pub = n.advertise<geometry_msgs::Vector3>("rotation", 1000);
    magnetism_pub = n.advertise<geometry_msgs::Vector3>("magnetism", 1000);
    euler_pub = n.advertise<geometry_msgs::Vector3>("euler", 1000);
   }


   MPU6050 imu;
   QMC5883 compass;
   MS5837_30BA bar;
   MadgwickFilter ahrs;

   imu.startup(i2c1, fd);
   compass.startup(i2c1, fd);
   bar.startup(i2c1, fd);
   bar.set_conversion(bar.Pa);

    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
    float axoffset, ayoffset, azoffset, gxoffset, gyoffset, gzoffset;
    float mxoffset, myoffset, mzoffset, mxscale, myscale, mzscale;

    while (ros::ok())
    {
        imu.get_raw_data();
        compass.get_raw_data();
        bar.get_raw_data();

        imu.Accelerometer::get_sample();
        imu.Gyroscope::get_sample();
        compass.get_sample();

        imu.Accelerometer::getX(ax, ay, az);
        imu.Gyroscope::getX(gx, gy, gz);
        compass.getX(mx, my, mz);

        ax = -ax; ay = -ay; az = -az;
        gx = -gx; gy = -gy; gz = -gz;
        mx = -mx; my = -my; mz = -mz;

        if (CALIBRATED)
        {
            n.getParam("/Accelerometer/axoffset", axoffset);
            n.getParam("/Accelerometer/ayoffset", ayoffset);
            n.getParam("/Accelerometer/azoffset", azoffset);
            n.getParam("/Gyroscope/gxoffset", gxoffset);
            n.getParam("/Gyroscope/gyoffset", gyoffset);
            n.getParam("/Gyroscope/gzoffset", gzoffset);
            n.getParam("/Magnetometer/mxoffset", mxoffset);
            n.getParam("/Magnetometer/myoffset", myoffset);
            n.getParam("/Magnetometer/mzoffset", mzoffset);
            n.getParam("/Magnetometer/mxscale", mxscale);
            n.getParam("/Magnetometer/myscale", myscale);
            n.getParam("/Magnetometer/mzscale", mzscale);
            // std::cout << axoffset << " " << ayoffset << " " << azoffset << std::endl;
            ax = ax - axoffset;
            ay = ay - ayoffset;
            az = az - azoffset;
            gx = gx - gxoffset;
            gy = gy - gyoffset;
            gz = gz - gzoffset;
            mx = (mx - mxoffset) * mxscale;
            my = (my - myoffset) * myscale;
            mz = (mz - mzoffset) * mzscale;
        }


        ahrs.update(gx, gy, gz, ax, ay, az, -mx, my, mz);

        geometry_msgs::PoseStamped msg;

        if (DEBUG)
        {
            geometry_msgs::Vector3 acceleration_msg;
            geometry_msgs::Vector3 rotation_msg;
            geometry_msgs::Vector3 magnetism_msg;
            geometry_msgs::Vector3 euler_msg;

            acceleration_msg.x = ax;
            acceleration_msg.y = ay;
            acceleration_msg.z = az;

            rotation_msg.x = gx;
            rotation_msg.y = gy;
            rotation_msg.z = gz;

            magnetism_msg.x = mx;
            magnetism_msg.y = my;
            magnetism_msg.z = mz;

            euler_msg.x = ahrs.getRoll();
            euler_msg.y = ahrs.getPitch();
            euler_msg.z = ahrs.getYaw();

            acceleration_pub.publish(acceleration_msg);
            rotation_pub.publish(rotation_msg);
            magnetism_pub.publish(magnetism_msg);
            euler_pub.publish(euler_msg);
        }

        msg.pose.orientation.w = ahrs.q0;
        msg.pose.orientation.x = ahrs.q1;
        msg.pose.orientation.y = ahrs.q2;
        msg.pose.orientation.z = ahrs.q3;

        msg.pose.position.x = 0;
        msg.pose.position.y = 0;
        msg.pose.position.z = bar.getDepth();

        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
