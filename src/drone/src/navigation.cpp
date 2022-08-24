#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "drone/sensors.h"
#include "geometry_msgs/Vector3.h"


#define I2C_BUS "/dev/i2c-1"


int main(int argc, char **argv)
{

//    int fd;
//
//    if ((fd = open(I2C_BUS, O_RDWR)) < 0)
//    {
//        // Open port for reading and writing
//
//        fprintf(stderr, "Failed to open i2c bus %s\n", I2C_BUS);
//
//        exit(1);
//    }

//    I2C i2c1((char*) I2C_BUS);

    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<geometry_msgs::Vector3>("talker_topic", 1000);
    ros::Rate loop_rate(10);

//    MPU6050 imu;
//    imu.startup(i2c1, fd);

    while (ros::ok())
    {
//        imu.get_raw_data();
        geometry_msgs::Vector3 msg;
        msg.x = 1;
        msg.y = 2;
        msg.z = 3;
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
