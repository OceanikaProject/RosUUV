#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "drone/Joystick.h"
#include "drone/Powers.h"
#include "geometry_msgs/PoseStamped.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf/transform_datatypes.h>
#include <iostream>


using namespace std;
using namespace message_filters;


static inline int map(float x, int in_min, int, int in_max, int out_min, int out_max);
{
    return static_cast<int>((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}


void control_callback(const drone::JoystickConstPtr& joystick_msg, const geometry_msgs::PoseStampedConstPtr& navigation_msg)
{
    drone::Powers powers_msg;

    unsigned int pressed_buttons = joystick_msg->pressed_buttons;

    bool A_active = (pressed_buttons >> 0) & 0b0000000000000001;
    bool D_active = (pressed_buttons >> 3) & 0b0000000000000001;
    bool R_active = (pressed_buttons >> 8) & 0b0000000000000001;
    bool L_active = (pressed_buttons >> 7) & 0b0000000000000001;

    // if (A_active)
    // {
    // power_hand.data = 1148;
    // }
    // else if (D_active)
    // {
    // power_hand.data = 1832
    // }
    // else
    // {
    // power_hand.data = 1488;
    // }

    cout << joystick_msg->left_y << " | " << joystick_msg->left_x << " | " << joystick_msg->right_y << " | " << joystick_msg->right_x << endl;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "control");
    ros::NodeHandle nh;

    ros::Publisher pub_hand = nh.advertise<std_msgs::Int16>("hand_angle", 1);
    std_msgs::Int16 power_hand;

    Subscriber<drone::Joystick> joystick_subscriber(nh, "joystick_state", 1);
    Subscriber<geometry_msgs::PoseStamped> navigation_subscriber(nh, "talker_topic", 1);
    TimeSynchronizer<drone::Joystick, geometry_msgs::PoseStamped> ts(joystick_subscriber, navigation_subscriber, 10);
    ts.registerCallback(boost::bind(&control_callback, _1, _2));
    ros::spin();
    return 0;
}
