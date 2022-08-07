#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"


std_msgs::Int32MultiArray thrust;

int lx, ly, rx, ry;
// ros::NodeHandle n;
// ros::Publisher pub = n.advertise<std_msgs::Int32MultiArray>("engines_topic", 1);


long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void joystick_control_callback(const std_msgs::Int32MultiArray msg)
{
    thrust.data.clear();
    int lx = map(msg.data[0], 0, 1023, -100, 100);
    int ly = map(msg.data[1], 0, 1023, -100, 100);
    int rx = map(msg.data[2], 0, 1023, -100, 100);
    int ry = map(msg.data[3], 0, 1023, -100, 100);
    thrust.data.push_back(ly/2 + lx/2);
    thrust.data.push_back(ly/2 + lx/2);
    thrust.data.push_back(ry + rx);
    thrust.data.push_back(ry - rx);
    thrust.data.push_back(ly - lx);


    // lx, ly, rx, ry
    // lv, rv, lh, rh, bv
    // [ly/2 + lx/2, ly/2 + lx/2, ry + rx, ry - rx, ly - lx]
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "engines");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("esp", 1000, joystick_control_callback);
    ros::Publisher pub = n.advertise<std_msgs::Int32MultiArray>("engines_topic", 1);

    ros::Rate rate(10);

    while (ros::ok())
    {
    //     thrust.data.push_back(ly/2 + lx/2);
    //     thrust.data.push_back(ly/2 + lx/2);
    //     thrust.data.push_back(ry + rx);
    //     thrust.data.push_back(ry - rx);
    //     thrust.data.push_back(ly - lx);
        pub.publish(thrust);
        ros::spinOnce();
        rate.sleep();
   }

    // ros::spin();
    return 0;
}
