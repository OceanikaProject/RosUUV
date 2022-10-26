#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "drone/sensors.h"
#include "geometry_msgs/Vector3.h"
#include <fcntl.h>
#include "drone/MadgwickFilter.h"
#include "drone/ComplementaryFilter.h"

#include "sensor_msgs/MagneticField.h"
#include "drone/Depth.h"


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

    ros::init(argc, argv, "navigation_module");
    ros::NodeHandle n;
    // ros::Publisher pub = n.advertise<geometry_msgs::PoseStamped>("navigation_module", 2);
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("/imu_data", 2);
    ros::Publisher depth_pub = n.advertise<drone::Depth>("/depth", 2);

    ros::Rate loop_rate(100);

    const bool DEBUG = n.hasParam("/inertial_navigation/DEBUG");
    const bool CALIBRATED = n.hasParam("/inertial_navigation/CALIBRATED");
    

    // static ros::Publisher acceleration_pub;
    // static ros::Publisher rotation_pub;
    // static ros::Publisher magnetism_pub;
    static ros::Publisher mf_pub;
    static ros::Publisher euler_pub;
    

   if (DEBUG)
   {
        // acceleration_pub = n.advertise<geometry_msgs::Vector3>("acceleration", 1000);
        // rotation_pub = n.advertise<geometry_msgs::Vector3>("rotation", 1000);
        mf_pub = n.advertise<sensor_msgs::MagneticField>("/magnetic_field", 2);
        euler_pub = n.advertise<geometry_msgs::Vector3>("/euler", 1000);
   }


    MPU6050 imu(MPU6050::CONVERT_2G, MPU6050::CONVERT_500DEG);
    QMC5883 compass;
    MS5837_30BA bar;
    // MadgwickFilter ahrs;
    ComplementaryFilter ahrs(0.55);

   

    imu.startup(i2c1, fd);
    compass.startup(i2c1, fd);
    bar.startup(i2c1, fd);
    bar.set_conversion(bar.Pa);

    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
    float yaw;
    float axoffset, ayoffset, azoffset, gxoffset, gyoffset, gzoffset;
    float mxoffset, myoffset, mzoffset, mxscale, myscale, mzscale;

    float sampleFrequency, beta;

    double time_moment = ros::Time::now().toSec();
    double dt;

    while (ros::ok())
    {

        // n.getParam("/sampleFrequency", sampleFrequency);
        // n.getParam("/beta", beta);
        // ahrs.begin(sampleFrequency, beta);

        imu.get_binary_data();
        compass.get_binary_data();
        bar.get_binary_data();

        imu.Accelerometer::get_sample();
        imu.Gyroscope::get_sample();
        compass.get_sample();

        imu.Accelerometer::get_3d_magnitude(ax, ay, az);
        imu.Gyroscope::get_3d_magnitude(gx, gy, gz);
        compass.get_3d_magnitude(mx, my, mz);

        // ax = -ax; ay = -ay; az = -az;
        // gx = -gx; gy = -gy; gz = -gz;
        // mx = -mx; my = -my; mz = -mz;

        if (CALIBRATED)
        {
            // n.getParam("/Accelerometer/axoffset", axoffset);
            // n.getParam("/Accelerometer/ayoffset", ayoffset);
            // n.getParam("/Accelerometer/azoffset", azoffset);
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
            // ax = ax - axoffset;
            // ay = ay - ayoffset;
            // az = az - azoffset;
            gx = gx - gxoffset;
            gy = gy - gyoffset;
            gz = gz - gzoffset;
            mx = (mx - mxoffset) * mxscale;
            my = (my - myoffset) * myscale;
            mz = (mz - mzoffset) * mzscale;
        }

        dt = ros::Time::now().toSec() - time_moment;
        ahrs.update(gx, gy, gz, ax, ay, az, mx, my, mz, dt);
        time_moment = ros::Time::now().toSec();

        // geometry_msgs::PoseStamped msg;
        sensor_msgs::Imu imu_msg;
        drone::Depth depth_msg;

        if (DEBUG)
        {
            // geometry_msgs::Vector3 acceleration_msg;
            // geometry_msgs::Vector3 rotation_msg;
            // geometry_msgs::Vector3 magnetism_msg;
            sensor_msgs::MagneticField mf_msg;
            geometry_msgs::Vector3 euler_msg;

            // acceleration_msg.x = ax;
            // acceleration_msg.y = ay;
            // acceleration_msg.z = az;

            // rotation_msg.x = gx;
            // rotation_msg.y = gy;
            // rotation_msg.z = gz;

            // magnetism_msg.x = mx;
            // magnetism_msg.y = my;
            // magnetism_msg.z = mz;

            // float heading = atan2(my, mx) * 180 / 3.14;

            mf_msg.magnetic_field.x = mx;
            mf_msg.magnetic_field.y = my;
            mf_msg.magnetic_field.z = mz;

            euler_msg.x = ahrs.getRoll();
            euler_msg.y = ahrs.getPitch();
            yaw = -ahrs.getYaw();

            // cout << heading << " | " << yaw << endl;
            // if (yaw <= 0) yaw += 360; 
            euler_msg.z = yaw;

            // acceleration_pub.publish(acceleration_msg);
            // rotation_pub.publish(rotation_msg);
            mf_pub.publish(mf_msg);
            euler_pub.publish(euler_msg);
        }
        // cout << dt << endl;
        // cout << ahrs.q[0] << " | " << ahrs.q[1] << " | " << ahrs.q[2] << " | " << ahrs.q[3] << endl;

        imu_msg.orientation.w = ahrs.q[0];
        imu_msg.orientation.x = ahrs.q[1];
        imu_msg.orientation.y = ahrs.q[2];
        imu_msg.orientation.z = ahrs.q[3];

        imu_msg.orientation_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};

        imu_msg.angular_velocity.x = gx;
        imu_msg.angular_velocity.y = gy;
        imu_msg.angular_velocity.z = gz;

        imu_msg.angular_velocity_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};

        imu_msg.linear_acceleration.x = ax;
        imu_msg.linear_acceleration.y = ay;
        imu_msg.linear_acceleration.z = az;

        imu_msg.linear_acceleration_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};

        depth_msg.value = bar.getDepth();

        // msg.pose.orientation.w = ahrs.q[0];
        // msg.pose.orientation.x = ahrs.q[1];
        // msg.pose.orientation.y = ahrs.q[2];
        // msg.pose.orientation.z = ahrs.q[3];

        // msg.pose.position.x = 0;
        // msg.pose.position.y = 0;
        // msg.pose.position.z = bar.getDepth();

        // pub.publish(msg);
        depth_pub.publish(depth_msg);
        imu_pub.publish(imu_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
